package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.LoggingConstants.EndEffectorLogging;

public class EndEffector extends SubsystemBase {
	private TalonFX angleMotor = new TalonFX(EndEffectorConstants.anglePort);
	private TalonFX wheelMotor = new TalonFX(EndEffectorConstants.wheelPort);

	private CANcoder encoder = new CANcoder(EndEffectorConstants.encoderPort);

	private Angle targetAngle = EndEffectorConstants.maxAngle;

	private LoggedSubsystem logger = new LoggedSubsystem("End Effector");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	public EndEffector() {
		angleMotor.getConfigurator().apply(EndEffectorConstants.angleConfig);
		wheelMotor.getConfigurator().apply(EndEffectorConstants.wheelConfig);
	}

	public void setAngle(Angle angle) {
		targetAngle = angle;
		angleMotor.setControl(new PositionVoltage(angle));
	}

	public void runIntake(Voltage volts) {
		wheelMotor.setControl(new VoltageOut(volts));
	}

	public Angle getAngle() {
		return encoder.getAbsolutePosition().getValue();
	}

	@Override
	public void periodic() {
	}

	public void initLogs() {
		logger.addDouble("current angle", () -> getAngle().in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("target angle", () -> targetAngle.in(Degrees), EndEffectorLogging.Angle);

		logger.addDouble("wheel volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(), EndEffectorLogging.Wheel);

		loggedAngle = new LoggedFalcon("angle motor", logger, angleMotor, EndEffectorLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel motor", logger, wheelMotor, EndEffectorLogging.Wheel);

		logger.add(loggedAngle);
		logger.add(loggedWheel);
	}
}
