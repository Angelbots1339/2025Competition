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
	private boolean seen_coral = false;

	public IntakeCoral(EndEffector endeffector) {
		this.endEffector = endeffector;
		addRequirements(endeffector);
	}

	@Override
	public void initialize() {
		endEffector.setAngle(Degrees.of(98));
		endEffector.runIntake(EndEffectorConstants.coralIntakeVolts);
		intake_timer.reset();
	}

	@Override
	public void execute() {
		if (endEffector.hasCoral()) {
			seen_coral = true;
			intake_timer.start();
		}

		if (!endEffector.hasCoral() && seen_coral == true && !intake_timer.hasElapsed(0.05)) {
			intake_timer.reset();
			seen_coral = false;
		}
	}

	@Override
	public void end(boolean interrupted) {
		endEffector.hold();
		intake_timer.reset();
		seen_coral = false;
	}

	@Override
	public boolean isFinished() {
		return !endEffector.hasCoral() && seen_coral == true && intake_timer.hasElapsed(0.5);
	}
}
