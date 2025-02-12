package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.SlotConfigs;
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
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LoggingConstants.IntakeLogging;

public class Intake extends SubsystemBase {
	private TalonFX leftAngleMotor = new TalonFX(IntakeConstants.leftAngleMotorPort);
	private TalonFX rightAngleMotor = new TalonFX(IntakeConstants.rightAngleMotorPort);
	private TalonFX wheelMotor = new TalonFX(IntakeConstants.wheelMotorPort);

	private Angle angle = IntakeConstants.insideAngle;

	private LoggedSubsystem logger = new LoggedSubsystem("Intake");
	private LoggedFalcon loggedLeftAngle;
	private LoggedFalcon loggedRightAngle;
	private LoggedFalcon loggedWheel;

	private Mechanism2d intake = new Mechanism2d(Units.inchesToMeters(26), Units.inchesToMeters(35));
	private MechanismLigament2d slapdown;

	public Intake() {
		leftAngleMotor.getConfigurator().apply(IntakeConstants.angleConfigs);
		rightAngleMotor.getConfigurator().apply(IntakeConstants.baseAngleConfigs);
		wheelMotor.getConfigurator().apply(IntakeConstants.wheelConfigs);

		rightAngleMotor.setControl(new Follower(leftAngleMotor.getDeviceID(), true));
		leftAngleMotor.setPosition(IntakeConstants.startingAngle);

		// changeAngle(() -> IntakeConstants.insideAngle);
		resetAngle(IntakeConstants.startingAngle);

		initLogging();

		setMech();
	}

	public void resetAngle(Angle angle) {
		leftAngleMotor.setPosition(angle);
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

	public void changeAngle(Supplier<Angle> angle) {
		this.angle = angle.get();
		leftAngleMotor.setControl(new PositionVoltage(angle.get()));
	}

	public Angle getAngle() {
		return leftAngleMotor.getPosition().getValue();
	}

	public Angle getAngleError() {
		return Rotations.of(leftAngleMotor.getClosedLoopError().getValueAsDouble());
	}

	public boolean isAtSetpoint() {
		return Math.abs(getAngleError().in(Degrees)) < IntakeConstants.angleErrorTolerence.in(Degrees);
	}

	@Override
	public void periodic() {
		slapdown.setAngle(angle.minus(Degrees.of(90)).in(Degrees));
	}

	public void setPID(SlotConfigs newPID) {
		leftAngleMotor.getConfigurator().apply(newPID);
	}

	public void setMech() {
		slapdown = intake.getRoot("Intake", Units.inchesToMeters(24.685), 0)
			.append(new MechanismLigament2d("Base", Units.inchesToMeters(11), 90))
			.append(new MechanismLigament2d("Slapdown", Units.inchesToMeters(20.012), 0, 6, new Color8Bit(edu.wpi.first.wpilibj.util.Color.kRed)));
		SmartDashboard.putData("Intake Mech", intake);
	}

	public void initLogging() {
		logger.addDouble("Current Angle", () -> getAngle().in(Degrees), IntakeLogging.Angle);
		logger.addDouble("Target Angle", () -> angle.in(Degrees), IntakeLogging.Angle);

		logger.addBoolean("At Setpoint", () -> isAtSetpoint(), IntakeLogging.Angle);
		logger.addDouble("Angle Error", () -> getAngleError().in(Degrees), IntakeLogging.Angle);
		logger.addDouble("Wheel Volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(), IntakeLogging.Wheel);


		loggedLeftAngle = new LoggedFalcon("Left Angle Motor", logger, leftAngleMotor, IntakeLogging.Angle);
		loggedRightAngle = new LoggedFalcon("Right Angle Motor", logger, rightAngleMotor, IntakeLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel Motor", logger, wheelMotor, IntakeLogging.Wheel);
		logger.add(loggedLeftAngle);
		logger.add(loggedRightAngle);
		logger.add(loggedWheel);
	}
}
