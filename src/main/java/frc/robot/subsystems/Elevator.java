package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private TalonFX motor = new TalonFX(ElevatorConstants.MotorPort);
	private double targetHeight = 0;

	private Mechanism2d mech = new Mechanism2d(Units.inchesToMeters(10), Units.feetToMeters(8));
	private MechanismLigament2d elevator;

	public Elevator() {
		motor.getConfigurator().apply(ElevatorConstants.config);

		motor.setPosition(0);

		elevator = mech.getRoot("base", Units.inchesToMeters(5), 0).append(new MechanismLigament2d("Stage 1", Units.inchesToMeters(8 * 12.0 / 3.0), 90));
		SmartDashboard.putData("Elevator", mech);
	}

	public void setHeight(double meters) {
		motor.setControl(
			ElevatorConstants.PositionRequest.withPosition(ElevatorConstants.metersToRotations(meters)));
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

	public boolean isAtSetpoint() {
		return getHeight() < ElevatorConstants.ErrorTolerence;
	}

	@Override
	public void periodic() {
		elevator.setLength(targetHeight);

		SmartDashboard.putNumber("Actual Height", getHeight());
		SmartDashboard.putNumber("Target Height", targetHeight);
		SmartDashboard.putBoolean("At Setpoint", isAtSetpoint());
	}
}
