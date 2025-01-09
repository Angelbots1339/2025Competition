// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final XboxController m_joystick = new XboxController(0);

	private Swerve swerve = new Swerve();

	private final Supplier<Double> leftX = () -> -m_joystick.getLeftX();
	private final Supplier<Double> leftY = () -> -m_joystick.getLeftY();
	private final Supplier<Double> rightX = () -> -m_joystick.getRightX();

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		swerve.setDefaultCommand(Commands.run(() -> {
			swerve.fieldCentricDrive(leftY, leftX, rightX);
		}, swerve));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
