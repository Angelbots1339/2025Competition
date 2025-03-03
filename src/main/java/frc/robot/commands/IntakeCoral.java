// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoral extends Command {
	private EndEffector endEffector;
  public IntakeCoral(EndEffector endeffector) {
	  this.endEffector = endeffector;
	  addRequirements(endeffector);
  }

  @Override
  public void initialize() {
	endEffector.runIntake(EndEffectorConstants.outtakeVolts);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
	  endEffector.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return endEffector.hasCoral();
  }
}
