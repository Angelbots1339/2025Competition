// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.tuning.IntakeTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TuningConstants.TuningSystem;
import frc.robot.subsystems.Intake;
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

	private final Intake intake = new Intake();
	/* IMPORTANT: Instantiate swerve subsystem last or else all other logging fails for some reason */
	private final Swerve swerve = new Swerve();

	private Trigger resetGyro = new Trigger(() -> driver.getStartButtonPressed());

	private Trigger alignClosestReef = new Trigger(() -> driver.getXButton());
	private Trigger alignSelectedReef = new Trigger(() -> driver.getBButton());
	private Trigger alignCoralStation = new Trigger(() -> driver.getYButton());
	private Trigger alignBargeCenter = new Trigger(() -> driver.getAButton());

	private Trigger alignProcessor = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);

	private Trigger selectReef = new Trigger(() -> driver.getPOV() != -1);

	private Trigger openIntake = new Trigger(() -> driver.getRightTriggerAxis() > 0.1);


	private final SendableChooser<Command> autoChooser;

	private final SendableChooser<TuningSystem> tuningChooser = new SendableChooser<>();

	public RobotContainer() {
		configureBindings();
		setDefaultCommands();

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData("Auto", autoChooser);

		for (TuningSystem system : TuningSystem.values()) {
			tuningChooser.addOption(system.toString(), system);
		}
		tuningChooser.setDefaultOption("None", TuningSystem.None);
		SmartDashboard.putData("Tuning System", tuningChooser);

	}

	private void configureBindings() {
		openIntake.whileTrue(intake.runIntake(() -> IntakeConstants.insideAngle.minus(Degree.of(90 * driver.getRightTriggerAxis()))));

		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve));

		alignClosestReef.whileTrue(swerve.defer(() -> swerve.driveToClosestReef()));
		alignSelectedReef.whileTrue(swerve.defer(swerve::driveToSelectedReef));

		alignCoralStation.whileTrue(swerve.defer(() -> swerve.driveToClosestCoralStation()));
		alignBargeCenter.whileTrue(swerve.defer(() -> swerve.driveToClosestBarge()));

		alignProcessor.whileTrue(swerve.defer(() -> swerve.driveToProcessor()));


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
					// TODO: there has to be a better way
					swerve.driveToSelectedReef(reef).onlyWhile(alignSelectedReef).schedule();
				}, swerve));

	}

	public void setDefaultCommands() {
		intake.setDefaultCommand(new InstantCommand(intake::home, intake));

		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true));
	}

	public void stopDefaultCommands() {
		intake.removeDefaultCommand();
		swerve.removeDefaultCommand();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public Command getTuningCommand() {
		return Commands.select(
			Map.ofEntries(
				Map.entry(TuningSystem.Intake, new IntakeTuning(intake)),
				Map.entry(TuningSystem.None, Commands.none())
			),
			() -> tuningChooser.getSelected());
	}
}
