package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.LoggingConstants.ElevatorLogging;

public class Elevator extends SubsystemBase {
	private TalonFX leader = new TalonFX(ElevatorConstants.LeaderPort);
	private TalonFX follower = new TalonFX(ElevatorConstants.FollowerPort);
	private double targetHeight = 0.0;

	private LoggedSubsystem logger = new LoggedSubsystem("Elevator");
	private LoggedFalcon loggedLeader;
	private LoggedFalcon loggedFollower;

	private Mechanism2d mech = new Mechanism2d(Units.inchesToMeters(20), Units.feetToMeters(8));
	private MechanismLigament2d base;
	private MechanismLigament2d stage1;

	public Elevator() {
		leader.getConfigurator().apply(ElevatorConstants.leaderConfigs);
		follower.getConfigurator().apply(ElevatorConstants.baseConfig);

		follower.setControl(new Follower(leader.getDeviceID(), true));

		reset();

		base = mech.getRoot("Elevator", Units.inchesToMeters(13.970), 0).append(new MechanismLigament2d("Base", ElevatorConstants.BaseHeight, 90));
		stage1 = base.append(new MechanismLigament2d("Stage 1", Units.inchesToMeters(1), 0, 6, new Color8Bit(Color.kRed)));
		SmartDashboard.putData("Elevator Mech", mech);
		initLogging();
	}

	public void reset() {
		leader.setPosition(0);
		follower.setPosition(0);
	}

	public void setHeight(double meters) {
		leader.setControl(ElevatorConstants.PositionRequest.withPosition(ElevatorConstants.metersToRotations(meters)));
		targetHeight = meters;
	}

	public void setHeight(Supplier<Double> meters) {
		leader.setControl(ElevatorConstants.PositionRequest.withPosition(ElevatorConstants.metersToRotations(meters.get())));
		targetHeight = meters.get();
	}

	public Command setHeightCommand(double meters) {
		return run(() -> setHeight(meters));
	}

	public Command setHeightCommand(Supplier<Double> meters) {
		return run(() -> setHeight(meters));
	}

	public double getHeight() {
		// if (RobotBase.isSimulation())
		// 	return targetHeight;
		return ElevatorConstants.rotationToMeters(getRotations());
	}

	public double getRotations() {
		return leader.getPosition().getValueAsDouble();
	}

	public double getErrorMeters() {
		return ElevatorConstants.rotationToMeters(leader.getClosedLoopError().getValueAsDouble());
	}

	public boolean isAtSetpoint() {
		return Math.abs(getErrorMeters()) <= ElevatorConstants.ErrorTolerence;
	}

	public void setPID(Slot0Configs tmp) {
		leader.getConfigurator().apply(tmp);
		follower.getConfigurator().apply(tmp);
	}

	public void setMotion(MotionMagicConfigs tmp) {
		leader.getConfigurator().apply(tmp);
		follower.getConfigurator().apply(tmp);
	}

	@Override
	public void periodic() {
		double length = targetHeight - ElevatorConstants.BaseHeight;
		// if (length < ElevatorConstants.BaseHeight) {
		// 	stage1.setLength(Units.inchesToMeters(1));
		// } else {
			stage1.setLength(length);
		// }
	}

	public void initLogging() {
		logger.addDouble("Target Height", () -> targetHeight, ElevatorLogging.Leader);
		logger.addDouble("Actual Height", this::getHeight, ElevatorLogging.Leader);

		logger.addDouble("Error", () -> getErrorMeters(), ElevatorLogging.Leader);
		logger.addBoolean("At Setpoint", () -> isAtSetpoint(), ElevatorLogging.Leader);

		loggedLeader = new LoggedFalcon("leader", logger, leader, ElevatorLogging.LeaderMotor);
		loggedFollower = new LoggedFalcon("follower", logger, follower, ElevatorLogging.FollowerMotor);
		logger.add(loggedLeader);
		logger.add(loggedFollower);
	}
}
