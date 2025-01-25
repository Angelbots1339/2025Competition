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
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final XboxController driver = new XboxController(DriverConstants.driverPort);
	private final XboxController operator = new XboxController(DriverConstants.operatorPort);

	private final Supplier<Double> leftX = () -> DriverConstants.deadbandJoystickValues(-driver.getLeftX(),
			SwerveConstants.maxspeed);
	private final Supplier<Double> leftY = () -> DriverConstants.deadbandJoystickValues(-driver.getLeftY(),
			SwerveConstants.maxspeed);
	private final Supplier<Double> rightX = () -> DriverConstants.deadbandJoystickValues(-driver.getRightX(),
			SwerveConstants.maxturn);

	private Swerve swerve = new Swerve();

	private Trigger resetGyro = new Trigger(() -> driver.getYButtonPressed());

	private Trigger alignToClosestReef = new Trigger(() -> driver.getXButton());
	private Trigger alignToSelectedReef = new Trigger(() -> driver.getBButton());
	private Trigger alignCoralStation = new Trigger(() -> driver.getRightTriggerAxis() > 0);
	private Trigger alignBargeCenter = new Trigger(() -> driver.getAButton());

	private Trigger selectReef = new Trigger(() -> driver.getPOV() != -1);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData(autoChooser);
	}

	private void configureBindings() {
		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve));

		alignToSelectedReef.whileTrue(swerve.defer(() -> swerve.driveToSelectedReef()));
		alignToClosestReef.whileTrue(swerve.defer(() -> swerve.driveToClosestReef()));

		alignCoralStation.whileTrue(swerve.defer(() -> swerve.driveToClosestCoralStation()));
		alignBargeCenter.whileTrue(swerve.defer(() -> swerve.driveToClosestBarge()));


		selectReef.onTrue(
				Commands.runOnce(() -> {
					int reef = 0;
					switch (driver.getPOV()) {
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
				}, swerve));

		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
