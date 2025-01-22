// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final XboxController m_joystick = new XboxController(0);
	private final Supplier<Double> leftX = () -> DriverConstants.deadbandJoystickValues(-m_joystick.getLeftX(), SwerveConstants.maxspeed);
	private final Supplier<Double> leftY = () -> DriverConstants.deadbandJoystickValues(-m_joystick.getLeftY(), SwerveConstants.maxspeed);
	private final Supplier<Double> rightX = () -> DriverConstants.deadbandJoystickValues(-m_joystick.getRightX(), SwerveConstants.maxturn);

	private Swerve swerve = new Swerve();

	private Trigger resetGyro = new Trigger(() -> m_joystick.getYButtonPressed());

	private Trigger moveToClosestReef = new Trigger(() -> m_joystick.getStartButton());
	private Trigger moveToSelectedReef = new Trigger(() -> m_joystick.getBackButton());

	private Trigger leftCoralStation = new Trigger(() -> m_joystick.getLeftBumperButton());
	private Trigger rightCoralStation = new Trigger(() -> m_joystick.getRightBumperButton());

	private Trigger selectReef = new Trigger(() -> m_joystick.getPOV() != -1);

	private Trigger moveForward = new Trigger(() -> m_joystick.getPOV() == 0);
	private Trigger moveBackward = new Trigger(() -> m_joystick.getPOV() == 180);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve).andThen(Commands.runOnce(() -> swerve.resetPose(Pose2d.kZero), swerve)));

		leftCoralStation.whileTrue(Commands.deferredProxy(() -> swerve.driveToLeftCoralStation()));
		rightCoralStation.whileTrue(Commands.deferredProxy(() -> swerve.driveToRightCoralStation()));

		moveToSelectedReef.whileTrue(Commands.deferredProxy(() -> swerve.driveToSelectedReef()));
		moveToClosestReef.whileTrue(Commands.deferredProxy(() -> swerve.driveToClosestReef()));

		moveForward.whileTrue(Commands.run(() -> swerve.drive(() -> 0.1, ()-> 0.0, () -> 0.0, false), swerve));
		moveBackward.whileTrue(Commands.run(() -> swerve.drive(() -> -0.1, ()-> 0.0, () -> 0.0, false), swerve));


/* 		selectReef.onTrue(
				Commands.runOnce(() -> {
					int reef = 0;
					switch (m_joystick.getPOV()) {
						case 0:
							reef = 0;
							break;
						case 45:
							reef = 5;
							break;
						case 135:
							reef = 4;
							break;
						case 180:
							reef = 3;
							break;
						case 225:
							reef = 2;
							break;
						case 315:
							reef = 1;
							break;
						default:
							return;
					}
					swerve.selectReef(reef);
				}, swerve)
	 	); */

		swerve.setDefaultCommand(Commands.run(() -> {
			swerve.drive(leftY, leftX, rightX, true);
		}, swerve));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
