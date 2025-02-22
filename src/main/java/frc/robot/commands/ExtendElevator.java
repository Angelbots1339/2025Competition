package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private Intake intake;

	private SequencingConstants.Heights target;

	public ExtendElevator(Elevator elevator, Intake intake, SequencingConstants.Heights target) {
		this.elevator = elevator;
		this.intake = intake;
		this.target = target;

		addRequirements(elevator, intake);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (target == SequencingConstants.Heights.A1 || target == SequencingConstants.Heights.A2) {
			intake.setAngle(SequencingConstants.reefAvoidAngle);
		} else {
			intake.setAngle(SequencingConstants.intakeAvoidAngle);
		}
		if (intake.isAtSetpoint()) {
			elevator.setHeight(target.height);
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (target != SequencingConstants.Heights.Barge) {
			intake.setAngle(IntakeConstants.insideAngle);
		}
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint() && intake.isAtSetpoint();
	}
}
