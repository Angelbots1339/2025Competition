// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command {
	private EndEffector endEffector;
	private Timer intake_timer = new Timer();

	public IntakeCoral(EndEffector endeffector) {
		this.endEffector = endeffector;
		addRequirements(endeffector);
	}

	@Override
	public void initialize() {
		endEffector.setAngle(Degrees.of(93));
		endEffector.runIntake(EndEffectorConstants.coralIntakeVolts);
		intake_timer.reset();
	}

	@Override
	public void execute() {
		if (endEffector.hasCoral()) {
			intake_timer.start();
		}
	}

	@Override
	public void end(boolean interrupted) {
		endEffector.hold();
		intake_timer.reset();
	}

	@Override
	public boolean isFinished() {
		return endEffector.hasCoral() && intake_timer.hasElapsed(0.2);
	}
}
