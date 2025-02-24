package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

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

	private TimeOfFlight sensor = new TimeOfFlight(EndEffectorConstants.sensorPort);

	private Angle targetAngle = EndEffectorConstants.maxAngle;

	private LoggedSubsystem logger = new LoggedSubsystem("End Effector");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	public EndEffector() {
		angleMotor.getConfigurator().apply(EndEffectorConstants.angleConfig);
		wheelMotor.getConfigurator().apply(EndEffectorConstants.wheelConfig);

		sensor.setRangingMode(RangingMode.Short, 24);
		initLogs();
	}

	public void home() {
		setAngle(EndEffectorConstants.defaultAngle);
		runIntake(Volts.zero());
	}

	public void intake(Angle angle) {
		setAngle(angle);
		runIntake(EndEffectorConstants.intakeVolts);
	}

	public void setAngle(Angle angle) {
		targetAngle = angle;
		angleMotor.setControl(new MotionMagicVoltage(angle));
	}

	public void setAngle(Supplier<Angle> angle) {
		setAngle(angle.get());
	}

	public void runIntake(Voltage volts) {
		wheelMotor.setControl(new VoltageOut(volts));
	}

	public Angle getAngle() {
		return encoder.getAbsolutePosition().getValue();
	}

	public Angle getAngleError() {
		return targetAngle.minus(getAngle());
	}

	public boolean isAtSetpoint() {
		return getAngleError().abs(Degrees) <= EndEffectorConstants.angleErrorTolerence.in(Degrees);
	}

	public boolean hasAlgae() {
		return sensor.getRange() <= EndEffectorConstants.hasAlgaeThreshold;
	}

	public void setPID(SlotConfigs newPID) {
		angleMotor.getConfigurator().apply(newPID);
	}
	public void setMotion(MotionMagicConfigs tmp) {
		angleMotor.getConfigurator().apply(tmp);
	}

	@Override
	public void periodic() {
	}

	public void initLogs() {
		logger.addDouble("current angle", () -> getAngle().in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("target angle", () -> targetAngle.in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("angle error", () -> getAngleError().in(Degrees), EndEffectorLogging.Angle);
		logger.addBoolean("at setpoint", this::isAtSetpoint, EndEffectorLogging.Angle);

		logger.addDouble("TOF distance", () -> sensor.getRange(), EndEffectorLogging.TOF);
		logger.addBoolean("Has Algae", this::hasAlgae, EndEffectorLogging.TOF);

		logger.addDouble("wheel volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(), EndEffectorLogging.Wheel);

		loggedAngle = new LoggedFalcon("angle motor", logger, angleMotor, EndEffectorLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel motor", logger, wheelMotor, EndEffectorLogging.Wheel);

		logger.add(loggedAngle);
		logger.add(loggedWheel);
	}
}
