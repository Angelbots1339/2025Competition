package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private EndEffector endEffector;

	static public SequencingConstants.SetPoints target = SequencingConstants.SetPoints.Home;
	public SequencingConstants.SetPoints override = null;
	static public double heightOverride = -1;

	public ExtendElevator(Elevator elevator, EndEffector endeffector, SequencingConstants.SetPoints override) {
		this(elevator, endeffector);
		this.override = override;
	}

	public ExtendElevator(Elevator elevator, EndEffector endeffector) {
		this.elevator = elevator;
		this.endEffector = endeffector;

		addRequirements(elevator, endeffector);
	}

	@Override
	public void initialize() {
		endEffector.setAngle(SequencingConstants.endEffectorAvoidAngle);
	}

	@Override
	public void execute() {
		if (!endEffector.isAtSetpoint()) {
			return;
		}

		if (override != null)
			elevator.setHeight(override.height);
		// else if (heightOverride != -1)
		// 	elevator.setHeight(heightOverride);
		else
			elevator.setHeight(target.height);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint() && endEffector.isAtSetpoint() && (
				override != null ? elevator.getTargetHeight() == override.height : elevator.getTargetHeight() == target.height
		);
	}
}
