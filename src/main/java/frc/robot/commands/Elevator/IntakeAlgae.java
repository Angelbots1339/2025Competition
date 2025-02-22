package frc.robot.commands.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import frc.robot.commands.ExtendElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class IntakeAlgae extends ExtendElevator {
	Supplier<Angle> targetAngle;
	Intake intake;

	public IntakeAlgae(Elevator elevator, Intake intake, Supplier<Angle> targetAngle) {
		super(elevator, intake, 0, false);

		this.targetAngle = targetAngle;
		this.intake = intake;
	}

	@Override
	public void run() {
		intake.runIntake(targetAngle);
	}
}
