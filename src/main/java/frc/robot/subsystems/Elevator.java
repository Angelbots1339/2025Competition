package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private TalonFX motor = new TalonFX(ElevatorConstants.MotorPort);
	private double targetHeight = 0;

	public Elevator() {
		motor.getConfigurator().apply(ElevatorConstants.config);

		motor.setPosition(0);
	}

	public void setHeight(double meters) {
		motor.setControl(
			ElevatorConstants.PositionRequest.withPosition(ElevatorConstants.metersToRotations(meters)));
		targetHeight = meters;
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
		SmartDashboard.putNumber("Actual Height", getHeight());
		SmartDashboard.putNumber("Target Height", targetHeight);
		SmartDashboard.putBoolean("At Setpoint", isAtSetpoint());
	}
}
