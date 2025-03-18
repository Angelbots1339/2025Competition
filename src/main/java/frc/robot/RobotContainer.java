// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.io.SequenceInputStream;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AlignUtil;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.AlignUtil.Side;
import frc.lib.util.LimelightHelpers.RawFiducial;
import frc.lib.util.tuning.ElevatorTuning;
import frc.lib.util.tuning.EndEffectorTuning;
import frc.lib.util.tuning.SuperstructureTuning;
import frc.lib.util.tuning.SwerveTuning;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.TuningConstants.TuningSystem;
import frc.robot.LoggingConstants.RobotContainerLogging;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final LoggedSubsystem logger = new LoggedSubsystem("RobotContainer");
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

	/*
	 * IMPORTANT: Instantiate swerve subsystem last or else all other logging fails
	 * for some reason
	 */
	public final Swerve swerve = new Swerve();

	// DRIVER TRIGGERS
	private Trigger resetGyro = new Trigger(() -> driver.getStartButtonPressed());
	private Trigger alignClosestReef = new Trigger(() -> driver.getAButton());
	private Trigger extendElevator = new Trigger(() -> driver.getBButton());
	private Trigger homeElevator = new Trigger(() -> driver.getYButton());

	private Trigger openIntake = new Trigger(() -> driver.getLeftTriggerAxis() > 0.5);
	private Trigger outtake = new Trigger(() -> driver.getLeftBumperButton());

	private Trigger intakeCoral = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
	private Trigger outtakeCoral = new Trigger(() -> driver.getRightBumperButton());

	// Kyle's Dead code
	// private Trigger alignSelectedReef = new Trigger(() -> driver.getBButton());
	// private Trigger alignCoralStation = new Trigger(() -> driver.getYButton());
	// private Trigger alignBarge = new Trigger(() -> driver.getAButton());

	// private Trigger alignClosestReef = new Trigger(() -> driver.getXButton());
	// private Trigger alignSelectedReef = new Trigger(() -> driver.getBButton());
	// private Trigger alignCoralStation = new Trigger(() -> driver.getYButton());
	private Trigger alignBarge = new Trigger(() -> driver.getXButton());

	// private Trigger alignProcessor = new Trigger(() ->
	// driver.getLeftTriggerAxis() > 0.5);
	// private Trigger selectReef = new Trigger(() -> driver.getPOV() != -1);

	// OPERATOR TRIGGERS
	private Trigger extendToBarge = new Trigger(() -> operator.getYButton());
	private Trigger extendToA1 = new Trigger(() -> operator.getAButton());
	private Trigger extendToA2 = new Trigger(() -> operator.getBButton());
	private Trigger home = new Trigger(() -> operator.getXButton());
	private Trigger extendToL4 = new Trigger(() -> operator.getPOV() == 0);
	private Trigger extendToL3 = new Trigger(() -> operator.getPOV() == 270 || operator.getPOV() == 90);
	private Trigger extendToL2 = new Trigger(() -> operator.getPOV() == 180);

	private Trigger selectRight = new Trigger(() -> operator.getRightTriggerAxis() > 0.1);
	private Trigger selectLeft = new Trigger(() -> operator.getLeftTriggerAxis() > 0.1);
	// private Trigger extendToL1 = new Trigger(() -> operator.getPOV() == 90);

	private Trigger deAlgae = new Trigger(() -> operator.getStartButton());

	private final SendableChooser<Command> autoChooser;

	private final SendableChooser<TuningSystem> tuningChooser = new SendableChooser<>();

	private Command bargeExtend = new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Barge)
			.andThen(new InstantCommand(() -> endeffector.setAngle(SequencingConstants.endEffectorBargeAngle)));

	public RobotContainer() {
		configureDriverBindings();
		configureOperatorBindings();
		setDefaultCommands();
		initLogs();

		NamedCommands.registerCommand("Score L4",
			new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.L4)
				.andThen(endeffector.setAngleAndRun(EndEffectorConstants.coralOuttakeVolts, SequencingConstants.SetPoints.L4.angle).withTimeout(2)
					.until(() -> !endeffector.hasCoral())));

		NamedCommands.registerCommand("Low Algae",
			new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.A1)
				.andThen(new RunCommand(() -> endeffector.intake(SequencingConstants.reefAlgaeAngle)).raceWith(Commands.waitSeconds(0.3)))
				.andThen(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home)));
		NamedCommands.registerCommand("A2",
			new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.A2)
				.andThen(new RunCommand(() -> endeffector.intake(SequencingConstants.reefAlgaeAngle)).raceWith(Commands.waitSeconds(0.3)))
				.andThen(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home)));
		NamedCommands.registerCommand("Barge",
				new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Barge)
						.andThen(new InstantCommand(
								() -> endeffector.setAngle(SequencingConstants.endEffectorBargeAngle))));
		NamedCommands.registerCommand("Home",
				new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home));

		NamedCommands.registerCommand("Outtake",
					Commands.run(() -> endeffector.runIntake(EndEffectorConstants.outtakeVolts), endeffector).raceWith(Commands.waitSeconds(0.5)));

		autoChooser = AutoBuilder.buildAutoChooser("Mobility");
		SmartDashboard.putData("Auto", autoChooser);

		for (TuningSystem system : TuningSystem.values()) {
			tuningChooser.addOption(system.toString(), system);
		}
		tuningChooser.setDefaultOption("None", TuningSystem.None);
		SmartDashboard.putData("Tuning System", tuningChooser);
	}

	private void configureOperatorBindings() {
		selectLeft.onTrue(Commands.runOnce(() -> AlignUtil.selectedSide = Side.Left));
		selectRight.onTrue(Commands.runOnce(() -> AlignUtil.selectedSide = Side.Right));
		home.onTrue(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home));
		extendToBarge.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.Barge));
		extendToA1.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.A1));
		extendToA2.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.A2));
		extendToL4.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L4));
		extendToL3.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L3));
		extendToL2.onTrue(
				Commands.runOnce(() -> ExtendElevator.target = SequencingConstants.SetPoints.L2));
		// extendToL1.onTrue(
		// Commands.runOnce(() -> ExtendElevator.target =
		// SequencingConstants.SetPoints.L1)
		// );
		// deAlgae.onTrue(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.DeAlgae)
		deAlgae.onTrue(Commands.either(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.DeAlgae),
			elevator.setHeightCommand(SequencingConstants.SetPoints.DeAlgae.height).until(() -> elevator.isAtSetpoint()),
			() -> elevator.getHeight() > 0.175)
				.andThen(endeffector.setAngleAndRun(Volts.of(5), Degrees.of(0))));

	}

	private void configureDriverBindings() {
		extendElevator.onTrue(new ExtendElevator(elevator, endeffector)
				.andThen(
						Commands.select(
								Map.ofEntries(
										Map.entry(SequencingConstants.SetPoints.L4, new RunCommand(
												() -> endeffector.setAngle(SequencingConstants.SetPoints.L4.angle))),
										Map.entry(SequencingConstants.SetPoints.L3, new RunCommand(
												() -> endeffector.setAngle(SequencingConstants.SetPoints.L3.angle))),
										Map.entry(SequencingConstants.SetPoints.L2, new RunCommand(
												() -> endeffector.setAngle(SequencingConstants.SetPoints.L2.angle))),
										Map.entry(SequencingConstants.SetPoints.A1, new RunCommand(
												() -> endeffector.intake(SequencingConstants.SetPoints.A1.angle))),
										Map.entry(SequencingConstants.SetPoints.A2, new RunCommand(
												() -> endeffector.intake(SequencingConstants.SetPoints.A2.angle))),
										Map.entry(SequencingConstants.SetPoints.Barge, new InstantCommand(
												() -> endeffector.setAngle(SequencingConstants.endEffectorBargeAngle))),
										Map.entry(SequencingConstants.SetPoints.Intake, Commands.none()),
										Map.entry(SequencingConstants.SetPoints.Home, Commands.none())),
								() -> ExtendElevator.target)));
		homeElevator.onTrue(new ExtendElevator(elevator, endeffector, SequencingConstants.SetPoints.Home));

		openIntake.whileTrue(
				Commands.run(() -> endeffector.intake(EndEffectorConstants.intakeAngle), endeffector)
						.onlyIf(() -> elevator.isAtHome()));

		intakeCoral.whileTrue(new IntakeCoral(endeffector)
				.andThen(Commands.run(() -> endeffector.setAngle(Degrees.of(98)))).onlyIf(() -> elevator.isAtHome()));
		outtakeCoral.whileTrue(
				Commands.run(() -> endeffector.runIntake(EndEffectorConstants.coralOuttakeVolts), endeffector));

		outtake.whileTrue(
				Commands.runOnce(() -> endeffector.hold()).andThen(
						Commands.either(
								Commands.run(() -> endeffector.setAngle(EndEffectorConstants.processorAngle),
										endeffector),
								Commands.run(() -> endeffector.runIntake(EndEffectorConstants.outtakeVolts),
										endeffector),
								() -> elevator.isAtHome())))
				.onFalse(Commands.runOnce(() -> endeffector.runIntake(EndEffectorConstants.outtakeVolts), endeffector)
						.andThen(Commands.waitSeconds(EndEffectorConstants.outtakeTime)));

		resetGyro.onTrue(Commands.runOnce(swerve::resetGyro, swerve));

		// alignClosestReef.whileTrue(swerve.defer(() -> Commands.run(() ->
		// swerve.pidToPose(AlignUtil.offsetPose(AlignUtil.getClosestReef(),
		// AlignUtil.coralOffset)), swerve)));
		// alignClosestReef.whileTrue(swerve.defer(() ->
		// AlignUtil.driveToClosestReef()));
		// alignSelectedReef.whileTrue(swerve.defer(() ->
		// AlignUtil.driveToSelectedReef()));
		// alignCoralStation.whileTrue(swerve.defer(() ->
		// AlignUtil.driveToClosestCoralStation()));
		// alignBarge.whileTrue(swerve.defer(() ->
		// AlignUtil.driveToClosestBarge().andThen(swerve.angularDrive(() -> 0.0, () ->
		// leftX.get() * 0.5, () ->
		// AlignUtil.getClosestBarge().getRotation().plus(Rotation2d.k180deg), () ->
		// true))));
		// alignBarge.whileTrue(swerve.defer(() -> Commands.run(() ->
		// swerve.pidToPose(new Pose2d(AlignUtil.getClosestBarge().getX(),
		// PoseEstimation.getEstimatedPose().getY(),
		// AlignUtil.getClosestBarge().getRotation())))));
		alignBarge.whileTrue(
				Commands.either(
						swerve.defer(() -> AlignUtil.driveToClosestBarge(swerve))
								.andThen(swerve.angularDrive(() -> 0.0, () -> leftX.get() * 0.2,
										() -> AlignUtil.getClosestBarge().getRotation().rotateBy(Rotation2d.k180deg),
										() -> true)),
						swerve.defer(() -> AlignUtil.driveToProcessor(swerve))
								.andThen(swerve.angularDrive(() -> leftY.get() * 0.2, () -> 0.0,
										() -> FieldUtil.getProcessor().getRotation().rotateBy(Rotation2d.k180deg),
										() -> true)),
						() -> FieldUtil.isRedAlliance() ? PoseEstimation.getEstimatedPose().getY() < 4
								: PoseEstimation.getEstimatedPose().getY() > 4));
		alignClosestReef.whileTrue(
				swerve.defer(() -> AlignUtil.driveToClosestReef(swerve))
						.andThen(swerve.angularDrive(() -> leftY.get() * 0.2, () -> leftX.get() * 0.2,
								() -> AlignUtil.getClosestReef().getRotation().rotateBy(Rotation2d.k180deg),
								() -> false)));
		// alignProcessor.whileTrue(swerve.defer(() -> AlignUtil.driveToProcessor()));

		// selectReef.onTrue(
		// Commands.runOnce(() -> {
		// int reef = 0;
		// switch (driver.getPOV()) {
		// case 0:
		// reef = 0;
		// break;
		// case 45:
		// reef = 5;
		// break;
		// case 135:
		// reef = 4;
		// break;
		// case 180:
		// reef = 3;
		// break;
		// case 225:
		// reef = 2;
		// break;
		// case 315:
		// reef = 1;
		// break;
		// default:
		// return;
		// }
		// AlignUtil.selectReef(reef);

		// // TODO: there has to be a better way
		// if (alignSelectedReef.getAsBoolean()) {
		// AlignUtil.driveToSelectedReef(reef).schedule();
		// return;
		// }

		// if (alignClosestReef.getAsBoolean()) {
		// AlignUtil.driveToClosestReef().schedule();
		// return;
		// }

		// if (alignCoralStation.getAsBoolean()) {
		// AlignUtil.driveToClosestCoralStation().schedule();
		// return;
		// }

		// if (alignProcessor.getAsBoolean()) {
		// AlignUtil.driveToProcessor().schedule();
		// return;
		// }

		// if (alignBarge.getAsBoolean()) {
		// AlignUtil.driveToClosestBarge().schedule();
		// return;
		// }

		// }, swerve));
	}

	public void setDefaultCommands() {
		swerve.setDefaultCommand(swerve.drive(leftY, leftX, rightX, () -> true, () -> !elevator.isAtHome()));
		endeffector.setDefaultCommand(
				Commands.run(() -> endeffector.home(), endeffector).onlyIf(() -> elevator.isAtHome())
						.andThen(Commands.run(() -> endeffector.hold(), endeffector)));
	}

	public void initLogs() {
		logger.addBoolean("Is Left", () -> AlignUtil.selectedSide == Side.Left, RobotContainerLogging.Side);
		logger.addBoolean("Is Right", () -> AlignUtil.selectedSide == Side.Right, RobotContainerLogging.Side);
	}

	public void stopDefaultCommands() {
		swerve.removeDefaultCommand();
		elevator.removeDefaultCommand();
		endeffector.removeDefaultCommand();
	}

	public Command getAutonomousCommand() {
		return Commands.waitSeconds(0.250).andThen(autoChooser.getSelected());
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
						Map.entry(TuningSystem.None, Commands.none())),
				() -> tuningChooser.getSelected());
	}
}
