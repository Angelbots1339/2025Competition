// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.Swerve;

public class SwerveTuning extends Command {
	private Swerve swerve;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve Tuning");
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private Trigger isFieldRelative = new Trigger(() -> test.getBButton());
	private Trigger shouldMove = new Trigger(() -> test.getPOV() != -1);

	public SwerveTuning(Swerve swerve) {
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		shouldMove.whileTrue(swerve.drive(() -> Math.cos(test.getPOV() * Math.PI/180.0) * 0.1, () -> -Math.sin(test.getPOV() * Math.PI/180.0) * 0.1, () -> 0.0 , () -> isFieldRelative.getAsBoolean()));
		swerve.setDefaultCommand(swerve.drive(() -> 0.0, () -> 0.0, () -> 0.0, () -> false));
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
