package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private TalonFX motor = new TalonFX(ElevatorConstants.MotorPort);
	private Distance targetHeight = Meters.zero();

	private LoggedSubsystem logger = new LoggedSubsystem("Elevator");

	private Mechanism2d mech = new Mechanism2d(Units.inchesToMeters(10), Units.feetToMeters(8));
	private MechanismLigament2d elevator;

	public Elevator() {
		motor.getConfigurator().apply(ElevatorConstants.config);

		motor.setPosition(0);

		elevator = mech.getRoot("Origin", Units.inchesToMeters(5), 0).append(new MechanismLigament2d("Base", Units.inchesToMeters(8 * 12.0 / 3.0), 90))
			.append(new MechanismLigament2d("Stage 1", 0, 0));
		SmartDashboard.putData("Elevator", mech);

		initLogging();
	}

	public void setHeight(double meters) {
		motor.setPosition(ElevatorConstants.metersToRotations(meters));
		targetHeight = Meters.of(meters);
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
		elevator.setLength(targetHeight.in(Meters));
		elevator.setAngle(Rotation2d.kCW_90deg);
	}

	public void initLogging() {
		logger.addDouble("Target Height", () -> targetHeight.in(Meters), LoggingLevel.NETWORK_TABLES);
		logger.addDouble("Actual Height", this::getHeight, LoggingLevel.NETWORK_TABLES);
	}
}
