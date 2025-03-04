package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private EndEffector endEffector;

	static public SequencingConstants.Heights target = SequencingConstants.Heights.Home;
	public SequencingConstants.Heights override = null;
	public double heightOverride = -1;

	public ExtendElevator(Elevator elevator, EndEffector endeffector, SequencingConstants.Heights override) {
		this(elevator, endeffector);
		this.override = override;
	}

	public ExtendElevator(Elevator elevator, EndEffector endeffector, double override) {
		this(elevator, endeffector);
		this.heightOverride = override;
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
		else if (heightOverride != -1)
			elevator.setHeight(heightOverride);
		else
			elevator.setHeight(target.height);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint() && endEffector.isAtSetpoint();
	}
}
