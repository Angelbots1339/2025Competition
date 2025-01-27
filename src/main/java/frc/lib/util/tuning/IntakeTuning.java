// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.Swerve;

public class IntakeTuning extends Command {
	Swerve swerve;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve Tuning");
	private static ShuffleboardLayout pid = tab.getLayout("PID", BuiltInLayouts.kList);
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static GenericEntry p, i, d, g, s;

	public IntakeTuning(Swerve swerve) {
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
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
