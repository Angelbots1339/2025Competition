package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.LoggingConstants.EndEffectorLogging;

public class EndEffector extends SubsystemBase {
	private TalonFX angleMotor = new TalonFX(EndEffectorConstants.anglePort);
	private TalonFX wheelMotor = new TalonFX(EndEffectorConstants.wheelPort);

	private DutyCycleEncoder encoder = new DutyCycleEncoder(EndEffectorConstants.encoderPort, 1,
			EndEffectorConstants.encoderOffset);

	private TimeOfFlight sensor = new TimeOfFlight(EndEffectorConstants.sensorPort);
	private TimeOfFlight funnelSensor = new TimeOfFlight(10);

	private Angle targetAngle = EndEffectorConstants.maxAngle;
	private final Timer throughBoreTimer = new Timer();

	private LoggedSubsystem logger = new LoggedSubsystem("End Effector");
	private LoggedFalcon loggedAngle;
	private LoggedFalcon loggedWheel;

	public EndEffector() {
		angleMotor.getConfigurator().apply(EndEffectorConstants.angleConfig);
		wheelMotor.getConfigurator().apply(EndEffectorConstants.wheelConfig);

		encoder.setInverted(true);
		throughBoreTimer.start();
		sensor.setRangingMode(RangingMode.Short, 24);
		funnelSensor.setRangingMode(RangingMode.Short, 24);
		initLogs();
		angleMotor.setPosition(EndEffectorConstants.maxAngle);
	}

	public void home() {
		setAngle(EndEffectorConstants.defaultAngle);

		hold();
	}

	public void intake(Angle angle) {
		setAngle(angle);
		runIntake(EndEffectorConstants.intakeVolts);
	}

	public Command setAngleAndRun(Voltage volts, Angle angle) {
		return run(() -> {
			setAngle(angle);
			if (isAtSetpoint())
				runIntake(volts);
		});
	}

	public void setAngle(Angle angle) {
		if (angle == null)
			return;
		targetAngle = angle;
		angleMotor.setControl(new MotionMagicVoltage(targetAngle));
	}

	public void setAngle(Supplier<Angle> angle) {
		setAngle(angle.get());
	}

	public void runIntake(Voltage volts) {
		if (volts == null)
			return;
		wheelMotor.setControl(new VoltageOut(volts));
	}

	public void resetAngle(Angle angle) {
		angleMotor.setPosition(angle);
	}

	public Angle getAngle() {
		return angleMotor.getPosition().getValue();
	}

	public void stop() {
		angleMotor.setControl(new NeutralOut());
		wheelMotor.setControl(new NeutralOut());
	}

	public void hold() {
		if (hasCoral())
			// wheelMotor.setControl(new NeutralOut());
			runIntake(Volts.of(0.17));
		else
			runIntake(EndEffectorConstants.algaeHoldVoltage);
	}

	public Angle getEncoderAngle() {
		Angle rot = Rotations.of(encoder.get() / EndEffectorConstants.gearRatio);
		return rot;
	}

	public Angle getAngleError() {
		return targetAngle.minus(getAngle());
	}

	public boolean isAtSetpoint() {
		return getAngleError().abs(Degrees) <= EndEffectorConstants.angleErrorTolerence.in(Degrees);
	}

	public boolean hasCoral() {
		// return sensor.getRange() <= EndEffectorConstants.hasAlgaeThreshold && sensor.getRange() > 65; /* lower bound because rigging sometimes gets detected */
		return sensor.getRange() <= EndEffectorConstants.hasAlgaeThreshold && funnelSensor.getRange() >= EndEffectorConstants.hasAlgaeThreshold; /* lower bound because rigging sometimes gets detected */
	}

	public void setPID(SlotConfigs newPID) {
		angleMotor.getConfigurator().apply(newPID);
	}

	public void setMotion(MotionMagicConfigs tmp) {
		angleMotor.getConfigurator().apply(tmp);
	}

	public void resetToAbsolute() {
		if (!encoder.isConnected())
			return;
		angleMotor.setPosition(getEncoderAngle().minus(Degrees.of(47)));
	}

	@Override
	public void periodic() {
		if (throughBoreTimer.get() >= EndEffectorConstants.timeBeforeEncoderReset) {
			resetToAbsolute();
			throughBoreTimer.reset();
			throughBoreTimer.stop();
		}
	}

	public void initLogs() {
		logger.addDouble("true encoder", () -> encoder.get(), EndEffectorLogging.Angle);
		logger.addBoolean("encoder", () -> encoder.isConnected(), EndEffectorLogging.Angle);
		logger.addDouble("encoder angle", () -> getEncoderAngle().in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("current angle", () -> getAngle().in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("target angle", () -> targetAngle.in(Degrees), EndEffectorLogging.Angle);
		logger.addDouble("angle error", () -> getAngleError().in(Degrees), EndEffectorLogging.Angle);
		logger.addBoolean("at setpoint", this::isAtSetpoint, EndEffectorLogging.Angle);
		logger.addDouble("pid error", () -> Rotations.of(angleMotor.getClosedLoopError().getValue()).in(Degrees), EndEffectorLogging.Angle);

		logger.addDouble("TOF distance", () -> sensor.getRange(), EndEffectorLogging.TOF);
		logger.addDouble("Funnel TOF distance", () -> funnelSensor.getRange(), EndEffectorLogging.TOF);
		logger.addBoolean("Has Coral", this::hasCoral, EndEffectorLogging.TOF);


		logger.addDouble("wheel volts", () -> wheelMotor.getMotorVoltage().getValueAsDouble(),
				EndEffectorLogging.Wheel);

		loggedAngle = new LoggedFalcon("angle motor", logger, angleMotor, EndEffectorLogging.Angle);
		loggedWheel = new LoggedFalcon("wheel motor", logger, wheelMotor, EndEffectorLogging.Wheel);
	}
}
