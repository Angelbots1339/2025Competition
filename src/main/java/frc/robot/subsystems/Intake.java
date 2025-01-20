package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LoggingConstants.IntakeLogging;

public class Intake extends SubsystemBase {
	private TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorPort);
	private TalonFX wheelMotor = new TalonFX(IntakeConstants.wheelMotorPort);

	private Angle angle = IntakeConstants.insideAngle;

	private LoggedSubsystem logger = new LoggedSubsystem("Intake");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	public Intake() {
		angleMotor.getConfigurator().apply(IntakeConstants.angleConfigs);

		changeAngle(() -> IntakeConstants.insideAngle);

		initLogging();
	}

	public void changeAngle(Supplier<Angle> angle) {
		this.angle = angle.get();
		angleMotor.setPosition(angle.get());
	}

	public Command changeAngleCommand(Supplier<Angle> angle) {
		return run(() -> changeAngle(angle));
	}

	public void home() {
		changeAngle(() -> IntakeConstants.insideAngle);
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
