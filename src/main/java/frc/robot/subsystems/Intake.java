package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
	private TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorPort);
	private TalonFX wheelMotor = new TalonFX(IntakeConstants.wheelMotorPort);

	public Intake() {
		angleMotor.getConfigurator().apply(IntakeConstants.angleConfigs);
	}

	public void changeAngle(Angle angle) {
		angleMotor.setPosition(angle);
	}

	public void runWheelsVolts(Voltage volts) {
		wheelMotor.setControl(new VoltageOut(volts));
	}

	@Override
	public void periodic() {
	}
}
