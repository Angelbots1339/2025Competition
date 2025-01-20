package frc.robot.subsystems;

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
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private TalonFX motor = new TalonFX(ElevatorConstants.MotorPort);
	private double targetHeight = 0.0;

	private LoggedSubsystem logger = new LoggedSubsystem("Elevator");

	private Mechanism2d mech = new Mechanism2d(Units.inchesToMeters(20), Units.feetToMeters(8));
	private MechanismLigament2d base;
	private MechanismLigament2d stage1;

	public Elevator() {
		motor.getConfigurator().apply(ElevatorConstants.config);

		motor.setPosition(0);


		base = mech.getRoot("Elevator", Units.inchesToMeters(10.5), 0).append(new MechanismLigament2d("Base", ElevatorConstants.BaseHeight, 90));
		stage1 = base.append(new MechanismLigament2d("Stage 1", Units.inchesToMeters(1), 0, 6, new Color8Bit(Color.kRed)));
		SmartDashboard.putData("Elevator Mech", mech);
		initLogging();
	}

	public void setHeight(double meters) {
		motor.setPosition(ElevatorConstants.metersToRotations(meters));
		targetHeight = meters;
	}

	public Command setHeightCommand(double meters) {
		return run(() -> setHeight(meters));
	}

	public double getHeight() {
		return ElevatorConstants.rotationToMeters(getRotations());
	}

	public double getRotations() {
		return motor.getPosition().getValueAsDouble();
	}

	@Override
	public void periodic() {
		stage1.setLength(targetHeight - ElevatorConstants.BaseHeight);
	}

	public void initLogging() {
		logger.addDouble("Target Height", () -> targetHeight, LoggingLevel.NETWORK_TABLES);
		logger.addDouble("Actual Height", this::getHeight, LoggingLevel.NETWORK_TABLES);
	}
}
