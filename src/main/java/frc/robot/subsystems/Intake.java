package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LoggingConstants.IntakeLogging;

public class Intake extends SubsystemBase {
	/* left Motor (intake is on front) */
	private TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorPort);
	private TalonFX angleFollowerMotor = new TalonFX(IntakeConstants.angleMotorFollowerPort);
	private TalonFX wheelMotor = new TalonFX(IntakeConstants.wheelMotorPort);

	private Angle angle = IntakeConstants.insideAngle;

	private LoggedSubsystem logger = new LoggedSubsystem("Intake");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	private Mechanism2d intake = new Mechanism2d(Units.inchesToMeters(20), Units.inchesToMeters(30));
	private MechanismLigament2d slapdown;

	public Intake() {
		angleMotor.getConfigurator().apply(IntakeConstants.angleConfigs);
		angleFollowerMotor.getConfigurator().apply(IntakeConstants.angleConfigs.withFeedback(IntakeConstants.angleFollwerConfiguration));

		angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), true));

		changeAngle(() -> IntakeConstants.insideAngle);

		initLogging();

		setMech();
	}

	public void setMech() {
		slapdown = intake.getRoot("Intake", Units.inchesToMeters(10.5), 0).append(new MechanismLigament2d("Base", Units.inchesToMeters(10), 90)).append(new MechanismLigament2d("Slapdown", Units.inchesToMeters(20), 0));
		SmartDashboard.putData("Intake Mech", intake);
	}

	public void changeAngle(Supplier<Angle> angle) {
		this.angle = angle.get();
		angleMotor.setControl(new PositionVoltage(angle.get()));
	}

	public Command runIntake(Supplier<Angle> angle) {
		return run(
			() -> {
				changeAngle(angle);
				runWheelsVolts(IntakeConstants.intakeVolts);
			}
		);
	}

	public void home() {
		changeAngle(() -> IntakeConstants.insideAngle);
		runWheelsVolts(Volts.of(0));
	}

	public void runWheelsVolts(Voltage volts) {
		wheelMotor.setControl(new VoltageOut(volts));
	}

	public Angle getAngle() {
		return angleMotor.getPosition().getValue();
	}

	public Angle getAngleError() {
		return Degrees.of(angleMotor.getClosedLoopError().getValueAsDouble());
	}

	@Override
	public void periodic() {
		slapdown.setAngle(angle.minus(Degrees.of(90)).in(Degrees));
	}

	public void initLogging() {
		logger.addDouble("Current Angle", () -> getAngle().in(Degrees), IntakeLogging.Angle);
		logger.addDouble("Target Angle", () -> angle.in(Degrees), IntakeLogging.Angle);
		logger.addDouble("Angle Error", () -> getAngleError().in(Degrees), IntakeLogging.Angle);


		logger.addDouble("Wheel Volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(), IntakeLogging.Wheel);


		loggedAngle = new LoggedFalcon("Angle Motor", logger, angleMotor, IntakeLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel Motor", logger, wheelMotor, IntakeLogging.Wheel);
		logger.add(loggedAngle);
		logger.add(loggedWheel);
	}
}
