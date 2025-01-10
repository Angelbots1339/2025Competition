// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final XboxController m_joystick = new XboxController(0);
	private final Supplier<Double> leftX = () -> -m_joystick.getLeftX();
	private final Supplier<Double> leftY = () -> -m_joystick.getLeftY();
	private final Supplier<Double> rightX = () -> -m_joystick.getRightX();

	private Swerve swerve = new Swerve();

	private Trigger moveToClosestReef = new Trigger(() -> m_joystick.getAButton());
	private Trigger leftCoralStation = new Trigger(() -> m_joystick.getBButton());
	private Trigger rightCoralStation = new Trigger(() -> m_joystick.getXButton());


	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		moveToClosestReef.whileTrue(Commands.deferredProxy(() -> swerve.driveToClosestReef()));
		leftCoralStation.whileTrue(Commands.deferredProxy(() -> swerve.driveToLeftCoralStation()));
		rightCoralStation.whileTrue(Commands.deferredProxy(() -> swerve.driveToRightCoralStation()));

		swerve.setDefaultCommand(Commands.run(() -> {
			swerve.drive(leftY, leftX, rightX, true);
		}, swerve));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
