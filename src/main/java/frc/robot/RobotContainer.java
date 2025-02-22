// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AlignUtil;
import frc.lib.util.tuning.IntakeTuning;
import frc.lib.util.tuning.ElevatorTuning;
import frc.lib.util.tuning.SwerveTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TuningConstants.TuningSystem;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.Elevator.IntakeAlgae;
import frc.robot.subsystems.Elevator;
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
	private Elevator elevator = new Elevator();
	/* IMPORTANT: Instantiate swerve subsystem last or else all other logging fails for some reason */
	private final Swerve swerve = new Swerve();

	private Trigger resetGyro = new Trigger(() -> driver.getStartButtonPressed());

	private Trigger alignClosestReef = new Trigger(() -> driver.getXButton());
	private Trigger alignSelectedReef = new Trigger(() -> driver.getBButton());
	private Trigger alignCoralStation = new Trigger(() -> driver.getYButton());
	private Trigger alignBarge = new Trigger(() -> driver.getAButton());

	private Trigger alignProcessor = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);

	private Trigger selectReef = new Trigger(() -> driver.getPOV() != -1);
	private Trigger extendElevator = new Trigger(() -> operator.getYButton());

	private Trigger openIntake = new Trigger(() -> operator.getLeftTriggerAxis() > 0.1);


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
		openIntake.whileTrue(
			new IntakeAlgae(elevator, intake, () -> IntakeConstants.insideAngle.minus(Degrees.of(90 * operator.getLeftTriggerAxis())))
		);
		extendElevator.whileTrue(new ExtendElevator(elevator, intake, 0.3));

		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve));

		alignClosestReef.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestReef()));
		alignSelectedReef.whileTrue(swerve.defer(() -> AlignUtil.driveToSelectedReef()));

		alignCoralStation.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestCoralStation()));
		alignBarge.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestBarge().andThen(swerve.angularDrive(() -> 0.0, () -> leftX.get() * 0.5, () -> AlignUtil.getClosestBarge().getRotation().plus(Rotation2d.k180deg), () -> true))));

		alignProcessor.whileTrue(swerve.defer(() -> AlignUtil.driveToProcessor()));


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
					AlignUtil.selectReef(reef);

					// TODO: there has to be a better way
					if (alignSelectedReef.getAsBoolean()) {
						AlignUtil.driveToSelectedReef(reef).schedule();
						return;
					}

					if (alignClosestReef.getAsBoolean()) {
						AlignUtil.driveToClosestReef().schedule();
						return;
					}

					if (alignCoralStation.getAsBoolean()) {
						AlignUtil.driveToClosestCoralStation().schedule();
						return;
					}

					if (alignProcessor.getAsBoolean()) {
						AlignUtil.driveToProcessor().schedule();
						return;
					}

					if (alignBarge.getAsBoolean()) {
						AlignUtil.driveToClosestBarge().schedule();
						return;
					}

				}, swerve));

	}

	public void setDefaultCommands() {
		intake.setDefaultCommand(new InstantCommand(intake::home, intake).onlyIf(() -> elevator.isAtHome()));
		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true));
	}

	public void stopDefaultCommands() {
		intake.removeDefaultCommand();
		swerve.removeDefaultCommand();
		elevator.removeDefaultCommand();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public Command getTuningCommand() {
		return Commands.select(
			Map.ofEntries(
				Map.entry(TuningSystem.Intake, new IntakeTuning(intake)),
				Map.entry(TuningSystem.Elevator, new ElevatorTuning(elevator)),
				Map.entry(TuningSystem.Swerve, new SwerveTuning(swerve)),
				Map.entry(TuningSystem.None, Commands.none())
			),
			() -> tuningChooser.getSelected());
	}
}
