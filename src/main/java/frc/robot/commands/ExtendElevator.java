package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.io.SequenceInputStream;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ExtendElevator extends Command {
	private Elevator elevator;
	private Intake intake;

	private double height;
	private boolean bringbackin;

	public ExtendElevator(Elevator elevator, Intake intake, double height, boolean bringbackin) {
		this.elevator = elevator;
		this.intake = intake;
		this.height = height;
		this.bringbackin = bringbackin;

		addRequirements(elevator, intake);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		intake.setAngle(SequencingConstants.intakeAvoidAngle);
		// if (intake.getAngle().in(Degrees) < 50){
		// 	elevator.setHeight(height);
		// }
		if (intake.isAtSetpoint()) {
			elevator.setHeight(height);
		}
	}

	public void run() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return elevator.isAtSetpoint() && intake.isAtSetpoint();
	}
}
