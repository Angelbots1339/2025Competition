package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
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
	private Distance targetHeight = Meters.zero();

	private LoggedSubsystem logger = new LoggedSubsystem("Elevator");

	private Mechanism2d mech = new Mechanism2d(Units.inchesToMeters(20), Units.feetToMeters(8));
	private MechanismLigament2d base;
	private MechanismLigament2d stage1;

	public Elevator() {
		motor.getConfigurator().apply(ElevatorConstants.config);

		motor.setPosition(0);


		base = mech.getRoot("Elevator", Units.inchesToMeters(10.5), 0).append(new MechanismLigament2d("Base", ElevatorConstants.BaseHeight.in(Meters), 90));
		stage1 = base.append(new MechanismLigament2d("Stage 1", Units.inchesToMeters(1), 0, 6, new Color8Bit(Color.kRed)));
		SmartDashboard.putData("Elevator Mech", mech);
		initLogging();
	}

	public void setHeight(Distance dist) {
		motor.setPosition(ElevatorConstants.metersToRotations(dist.in(Meters)));
		targetHeight = dist;
	}

	public Command setHeightCommand(Distance dist) {
		return run(() -> setHeight(dist));
	}

	public double getHeight() {
		return ElevatorConstants.rotationToMeters(getRotations());
	}

	public double getRotations() {
		return motor.getPosition().getValueAsDouble();
	}

	@Override
	public void periodic() {
		stage1.setLength(targetHeight.minus(ElevatorConstants.BaseHeight).in(Meters));
	}

	public void initLogging() {
		logger.addDouble("Target Height", () -> targetHeight.in(Meters), LoggingLevel.NETWORK_TABLES);
		logger.addDouble("Actual Height", this::getHeight, LoggingLevel.NETWORK_TABLES);
	}
}
