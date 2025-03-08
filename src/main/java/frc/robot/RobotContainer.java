// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.SequenceInputStream;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.tuning.ElevatorTuning;
import frc.lib.util.tuning.EndEffectorTuning;
import frc.lib.util.tuning.SuperstructureTuning;
import frc.lib.util.tuning.SwerveTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TuningConstants.TuningSystem;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
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

	private Elevator elevator = new Elevator();
	private final EndEffector endeffector = new EndEffector();
	/* IMPORTANT: Instantiate swerve subsystem last or else all other logging fails for some reason */
	private final Swerve swerve = new Swerve();

	private Trigger resetGyro = new Trigger(() -> driver.getStartButtonPressed());

	private Trigger alignClosestReef = new Trigger(() -> driver.getXButton());
	private Trigger alignSelectedReef = new Trigger(() -> driver.getBButton());
	private Trigger alignCoralStation = new Trigger(() -> driver.getYButton());
	private Trigger alignBarge = new Trigger(() -> driver.getAButton());

	private Trigger alignProcessor = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);

	private Trigger selectReef = new Trigger(() -> driver.getPOV() != -1);

	private Trigger extendToBarge = new Trigger(() -> operator.getYButton());
	private Trigger extendToA1 = new Trigger(() -> operator.getAButton());
	private Trigger extendToA2 = new Trigger(() -> operator.getBButton());
	private Trigger home = new Trigger(() -> operator.getXButton());
	private Trigger extendToL4 = new Trigger(() -> operator.getPOV() == 0);
	private Trigger extendToL3 = new Trigger(() -> operator.getPOV() == 270);
	private Trigger extendToL2 = new Trigger(() -> operator.getPOV() == 90);
	private Trigger extendToL1 = new Trigger(() -> operator.getPOV() == 180);


	private Trigger extendElevator = new Trigger(() -> driver.getBButton());
	private Trigger homeElevator = new Trigger(() -> driver.getYButton());

	private Trigger openIntake = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
	private Trigger outtake = new Trigger(() -> driver.getLeftBumperButton());

	private Trigger intakeCoral = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
	private Trigger outtakeCoral = new Trigger(() -> driver.getRightBumperButton());
	private final SendableChooser<Command> autoChooser;

	private final SendableChooser<TuningSystem> tuningChooser = new SendableChooser<>();

	public RobotContainer() {
		configureDriverBindings();
		configureOperatorBindings();
		setDefaultCommands();

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData("Auto", autoChooser);

		NamedCommands.registerCommand("Score L4",
			new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.L4)
				.andThen(endeffector.setAngleAndRun(() ->EndEffectorConstants.coralOuttakeVolts, () -> SequencingConstants.SetPoints.L4.angle)
					.until(() -> !endeffector.hasCoral())));

		for (TuningSystem system : TuningSystem.values()) {
			tuningChooser.addOption(system.toString(), system);
		}
		tuningChooser.setDefaultOption("None", TuningSystem.None);
		SmartDashboard.putData("Tuning System", tuningChooser);
	}

	private void configureOperatorBindings() {
		home.onTrue(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home));
		extendToBarge.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.Barge)
		);
		extendToA1.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.A1)
		);
		extendToA2.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.A2)
		);
		extendToL4.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L4)
		);
		extendToL3.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L3)
		);
		extendToL2.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L2)
		);
		extendToL1.onTrue(
			Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L1)
		);

	}

	private void configureDriverBindings() {
		extendElevator.onTrue(new ExtendElevator(elevator, endeffector)
		.andThen(
		Commands.select(
			Map.ofEntries(
				Map.entry(SequencingConstants.SetPoints.L4, new RunCommand(() -> endeffector.setAngle(SequencingConstants.SetPoints.L4.angle))),
				Map.entry(SequencingConstants.SetPoints.A1, new RunCommand(() -> endeffector.intake(SequencingConstants.A1Angle))),
				Map.entry(SequencingConstants.SetPoints.A2, new RunCommand(() -> endeffector.intake(SequencingConstants.A2Angle))),
				Map.entry(SequencingConstants.SetPoints.Barge, new InstantCommand(() -> endeffector.setAngle(SequencingConstants.endEffectorBargeAngle))),
				Map.entry(SequencingConstants.SetPoints.Intake, Commands.none()),
				Map.entry(SequencingConstants.SetPoints.Home, Commands.none())
			),
			() -> ExtendElevator.target)));
		homeElevator.onTrue(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home));

		openIntake.whileTrue(
				Commands.run(() -> endeffector.intake(EndEffectorConstants.intakeAngle), endeffector).onlyIf(() -> elevator.isAtHome())
		);

		intakeCoral.whileTrue(new IntakeCoral(endeffector).onlyIf(() -> elevator.isAtHome()));
		outtakeCoral.whileTrue(Commands.run(() -> endeffector.runIntake(EndEffectorConstants.coralOuttakeVolts), endeffector));

		outtake.whileTrue(
				Commands.either(
					Commands.run(() -> endeffector.setAngle(EndEffectorConstants.processorAngle), endeffector),
					Commands.run(() -> endeffector.runIntake(EndEffectorConstants.outtakeVolts), endeffector),
				() -> elevator.isAtHome())
		).onFalse(Commands.runOnce(() -> endeffector.runIntake(EndEffectorConstants.outtakeVolts), endeffector)
					.andThen(Commands.waitSeconds(EndEffectorConstants.outtakeTime)));

		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve));

		// alignClosestReef.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestReef()));
		// alignSelectedReef.whileTrue(swerve.defer(() -> AlignUtil.driveToSelectedReef()));
		// alignCoralStation.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestCoralStation()));
		// alignBarge.whileTrue(swerve.defer(() -> AlignUtil.driveToClosestBarge().andThen(swerve.angularDrive(() -> 0.0, () -> leftX.get() * 0.5, () -> AlignUtil.getClosestBarge().getRotation().plus(Rotation2d.k180deg), () -> true))));
		// alignProcessor.whileTrue(swerve.defer(() -> AlignUtil.driveToProcessor()));

		// selectReef.onTrue(
		// 		Commands.runOnce(() -> {
		// 			int reef = 0;
		// 			switch (driver.getPOV()) {
		// 				case 0:
		// 					reef = 0;
		// 					break;
		// 				case 45:
		// 					reef = 5;
		// 					break;
		// 				case 135:
		// 					reef = 4;
		// 					break;
		// 				case 180:
		// 					reef = 3;
		// 					break;
		// 				case 225:
		// 					reef = 2;
		// 					break;
		// 				case 315:
		// 					reef = 1;
		// 					break;
		// 				default:
		// 					return;
		// 			}
		// 			AlignUtil.selectReef(reef);

		// 			// TODO: there has to be a better way
		// 			if (alignSelectedReef.getAsBoolean()) {
		// 				AlignUtil.driveToSelectedReef(reef).schedule();
		// 				return;
		// 			}

		// 			if (alignClosestReef.getAsBoolean()) {
		// 				AlignUtil.driveToClosestReef().schedule();
		// 				return;
		// 			}

		// 			if (alignCoralStation.getAsBoolean()) {
		// 				AlignUtil.driveToClosestCoralStation().schedule();
		// 				return;
		// 			}

		// 			if (alignProcessor.getAsBoolean()) {
		// 				AlignUtil.driveToProcessor().schedule();
		// 				return;
		// 			}

		// 			if (alignBarge.getAsBoolean()) {
		// 				AlignUtil.driveToClosestBarge().schedule();
		// 				return;
		// 			}

		// 		}, swerve));
	}

	public void setDefaultCommands() {
		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true, () -> !elevator.isAtHome()));
		endeffector.setDefaultCommand(
				Commands.run(() -> endeffector.home(), endeffector).onlyIf(() -> elevator.isAtHome())
				.andThen(Commands.run(() -> endeffector.hold(), endeffector)));
	}

	public void stopDefaultCommands() {
		swerve.removeDefaultCommand();
		elevator.removeDefaultCommand();
		endeffector.removeDefaultCommand();
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public Command getTuningCommand() {
		endeffector.stop();
		elevator.stop();
		ExtendElevator.heightOverride = -1;
		return Commands.select(
			Map.ofEntries(
				Map.entry(TuningSystem.Superstructure, new SuperstructureTuning(elevator, endeffector)),
				Map.entry(TuningSystem.EndEffector, new EndEffectorTuning(endeffector)),
				Map.entry(TuningSystem.Elevator, new ElevatorTuning(elevator)),
				Map.entry(TuningSystem.Swerve, new SwerveTuning(swerve)),
				Map.entry(TuningSystem.None, Commands.none())
			),
			() -> tuningChooser.getSelected());
	}
}
