package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ExtendElevator extends Command {
	Elevator elevator;
	Intake intake;

	double height;

	public ExtendElevator(Elevator elevator, Intake intake, double height) {
		this.elevator = elevator;
		this.intake = intake;
		this.height = height;
	}

	@Override
	public void initialize() {
		intake.setAngle(Degrees.of(80));
	}

	@Override
	public void execute() {
		if (!intake.isAtSetpoint())
			return;

		elevator.setHeight(height);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint();
	}
}
