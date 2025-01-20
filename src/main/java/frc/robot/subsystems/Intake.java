package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LoggingConstants.IntakeLogging;

public class Intake extends SubsystemBase {
	private TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorPort);
	private TalonFX wheelMotor = new TalonFX(IntakeConstants.wheelMotorPort);

	private Angle angle;

	private LoggedSubsystem logger = new LoggedSubsystem("Intake");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	public Intake() {
		angleMotor.getConfigurator().apply(IntakeConstants.angleConfigs);
	}

	public void changeAngle(Angle angle) {
		this.angle = angle;
		angleMotor.setPosition(angle);
	}

	public void runWheelsVolts(Voltage volts) {
		wheelMotor.setControl(new VoltageOut(volts));
	}

	public Angle getAngle() {
		return angleMotor.getPosition().getValue();
	}

	@Override
	public void periodic() {
		logger.addDouble("Target Angle", () -> angle.in(Degrees), IntakeLogging.Angle);
		logger.addDouble("Current Angle", () -> getAngle().in(Degrees), IntakeLogging.Angle);

		logger.addDouble("Wheel Volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(), IntakeLogging.Wheel);


		loggedAngle = new LoggedFalcon("Angle Motor", logger, angleMotor, IntakeLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel Motor", logger, wheelMotor, IntakeLogging.Wheel);
		logger.add(loggedAngle);
		logger.add(loggedWheel);
	}
}
