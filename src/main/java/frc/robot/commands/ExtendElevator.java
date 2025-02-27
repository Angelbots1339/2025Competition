package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private EndEffector endEffector;

	private SequencingConstants.Heights target;

	public ExtendElevator(Elevator elevator, EndEffector endeffector, SequencingConstants.Heights target) {
		this.elevator = elevator;
		this.endEffector = endeffector;
		this.target = target;

		addRequirements(elevator, endeffector);
	}

	@Override
	public void initialize() {
		// if (endEffector.hasAlgae()) {
		// 	intake.setAngle(SequencingConstants.algaeAvoidAngle);
		// } else {
		// 	intake.setAngle(SequencingConstants.intakeAvoidAngle);
		// }
		endEffector.setAngle(SequencingConstants.endEffectorAvoidAngle);
	}

	@Override
	public void execute() {
		// if (!intake.isAtSetpoint()) {
		// 	return;
		// }


		if (!endEffector.isAtSetpoint()) {
			return;
		}

		elevator.setHeight(target.height);
	}

	@Override
	public void end(boolean interrupted) {
		// if (target == SequencingConstants.Heights.Barge) {
		// 	intake.setAngle(IntakeConstants.minAngle);
		// 	return;
		// }

		// if (endEffector.hasAlgae())
		// 	intake.setAngle(IntakeConstants.algaeStayAngle);
		// else
		// 	intake.setAngle(IntakeConstants.maxAngle);
	}

	@Override
	public boolean isFinished() {
		// return elevator.isAtSetpoint() && intake.isAtSetpoint() && endEffector.isAtSetpoint();
		return elevator.isAtSetpoint() && endEffector.isAtSetpoint();
	}
}
