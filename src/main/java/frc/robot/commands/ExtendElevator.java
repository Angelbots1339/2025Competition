package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private Intake intake;

	private double height;

	public ExtendElevator(Elevator elevator, Intake intake, double height) {
		this.elevator = elevator;
		this.intake = intake;
		this.height = height;

		addRequirements(elevator, intake);
	}

	@Override
	public void initialize() {
		intake.setAngle(SequencingConstants.intakeAvoidAngle);
	}

	@Override
	public void execute() {
		if (!intake.isAtSetpoint())
			return;

		elevator.setHeight(height);
	}

	@Override
	public void end(boolean interrupted) {
		// if (Math.abs(elevator.getHeight() - SequencingConstants.IntakeHitPoint) > SequencingConstants.IntakeHitPointBound)
		// 	intake.setAngle(IntakeConstants.insideAngle);
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint();
	}
}
