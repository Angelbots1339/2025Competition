// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedSweveModules;
import frc.robot.Constants.RobotConstants;
import frc.robot.LoggingConstants.SwerveLogging;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.swerve;
	SwerveDrivePoseEstimator pose = new SwerveDrivePoseEstimator(swerve.getKinematics(), getRelativeYaw(), swerve.getState().ModulePositions, Pose2d.kZero);

	private double maxspeed = 3;
	private double maxturn = Math.PI * 2;

	private double coralScoreOffsetY = Units.inchesToMeters(0);
	// private double coralScoreOffsetX = Units.inchesToMeters(32.75);
	private double coralScoreOffsetX = 0.938 / 2;

	/* offset is relative to robot */
	private Translation2d processorOffset = new Translation2d(RobotConstants.length / 2, 0);

	private Translation2d bargeOffset = new Translation2d(Units.inchesToMeters(-24 / 2.0), 0.0);

	private Pose2d selectedReef = new Pose2d(0, 0, Rotation2d.kZero);

	private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
			.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.RobotCentric m_robotRequest = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private LoggedSubsystem logger = new LoggedSubsystem("Swerve");
	private LoggedSweveModules logged_modules;
	private LoggedField logged_field;

	private int selectedReefindex = -1;

	/** Creates a new Swerve. */
	public Swerve() {
		configPathPlanner();
		swerve.getPigeon2().setYaw(0);

		initlogs();
		//putSwerveState();
	}

	public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn,
			Supplier<Boolean> fieldRelative) {
		return run(() -> {
			ChassisSpeeds speeds = new ChassisSpeeds(x.get() * maxspeed, y.get() * maxspeed, turn.get() * maxturn);
			SwerveRequest req;

			if (fieldRelative.get()) {
				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());
			}

			req = m_robotRequest
					.withVelocityX(speeds.vxMetersPerSecond)
					.withVelocityY(speeds.vyMetersPerSecond)
					.withRotationalRate(speeds.omegaRadiansPerSecond);

			swerve.setControl(req);
		});
	}

	public void resetPose(Pose2d pose) {
		swerve.resetPose(pose);
	}

	void setChassisSpeeds(ChassisSpeeds speeds) {
		swerve.setControl(autoRequest.withSpeeds(speeds));
	}

	ChassisSpeeds getRobotRelativeSpeeds() {
		return swerve.getState().Speeds;
	}

	private void configPathPlanner() {
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}
		AutoBuilder.configure(
				PoseEstimation::getEstimatedPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT
																	// RELATIVE ChassisSpeeds. Also optionally
																	// outputs individual module feedforwards
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
												// holonomic drive trains
						new PIDConstants(4.285, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5, 0.0, 0.2857) // Rotation PID constants
				),
				config, // The robot configuration
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					// var alliance = DriverStation.getAlliance();
					// if (alliance.isPresent()) {
					// return alliance.get() == DriverStation.Alliance.Red;
					// }
					// return FieldUtil.isRedAlliance() && !DriverStation.isTeleop();
					return FieldUtil.isRedAlliance();
				},
				this // Reference to this subsystem to set requirements
		);
	}

	public Rotation2d getYaw() {
		return swerve.getPigeon2().getRotation2d();
	}

	public Rotation2d getRelativeYaw() {
		return FieldUtil.isRedAlliance() ? getYaw().rotateBy(Rotation2d.k180deg) : getYaw();
	}

	public void setYaw(Rotation2d angle) {
		swerve.getPigeon2().setYaw(angle.getDegrees());
	}

	public void resetGyro() {
		setYaw(FieldUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
	}

	public Rotation2d getModuleAngle(int i) {
		return swerve.getModule(i).getCurrentState().angle;
	}

	public double getModuleSpeed(int i) {
		return swerve.getModule(i).getCurrentState().speedMetersPerSecond;
	}

	public Pose2d getClosestReef() {
		double dist = 10000000;
		int close = 0;
		Pose2d[] reefs = FieldUtil.getReef();
		for (int i = 0; i < reefs.length; i++) {
			Pose2d reef = reefs[i];
			double distToReef = PoseEstimation.getEstimatedPose().getTranslation().getDistance(reef.getTranslation());
			if (distToReef < dist) {
				dist = distToReef;
				close = i;
			}
		}

		return reefs[close];
	}

	public Pose2d getReefScoreSpot(Pose2d reef) {
		Translation2d tmp = new Translation2d(coralScoreOffsetX, coralScoreOffsetY);
		tmp.rotateBy(reef.getRotation());

		reef = reef.plus(new Transform2d(tmp.getX(), tmp.getY(), Rotation2d.k180deg));
		return reef;
	}

	public void selectReef(int i) {
		selectedReef = FieldUtil.getReef()[i];
		selectedReefindex = i;
	}

	public Pose2d getSelectedReef() {
		return selectedReef;
	}

	public Command driveToPose(Pose2d target) {
		PathConstraints constraints = new PathConstraints(3.0, 4.0,
				maxturn, Units.degreesToRadians(720));
		return AutoBuilder.pathfindToPose(target, constraints, 0.0);
	}

	public Command driveToClosestReef() {
		return driveToPose(getReefScoreSpot(getClosestReef()));
	}

	public Command driveToSelectedReef() {
		return driveToSelectedReef(selectedReefindex);
	}

	public Command driveToSelectedReef(int i) {
		if (i < 0 || i > 5)
			return Commands.none();
		selectReef(i);
		if (selectedReef.equals(new Pose2d(0, 0, Rotation2d.kZero)))
			return Commands.none();

		return driveToPose(getReefScoreSpot(selectedReef));
	}

	public Command driveToLeftCoralStation() {
		return driveToPose(FieldUtil.getLeftCoralStation());
	}

	public Command driveToRightCoralStation() {
		return driveToPose(FieldUtil.getRightCoralStation());
	}

	public Pose2d getClosestBarge() {
		Pose2d target = PoseEstimation.getEstimatedPose()
				.nearest(List.of(FieldUtil.getAllianceSideBargeCenter(), FieldUtil.getOpponateSideBargeCenter()));
		return target.plus(new Transform2d(bargeOffset, Rotation2d.kZero));
	}

	public Pose2d getClosestCoralStations() {
		Pose2d target = PoseEstimation.getEstimatedPose()
				.nearest(List.of(FieldUtil.getLeftCoralStation(), FieldUtil.getRightCoralStation()));
		return target;
	}

	public Command driveToClosestCoralStation() {
		return driveToPose(getClosestCoralStations());
	}

	public Command driveToClosestBarge() {
		return driveToPose(getClosestBarge());
	}

	public Command driveToProcessor() {
		Translation2d tmp = processorOffset;
		tmp.rotateBy(FieldUtil.getProcessor().getRotation());

		Pose2d processorScorePos = FieldUtil.getProcessor().plus(new Transform2d(tmp.getX(), tmp.getY(), Rotation2d.k180deg));

		return driveToPose(processorScorePos);
	}

	@Override
	public void periodic() {
			updateVision();
		pose.update(getYaw(), swerve.getState().ModulePositions);
		PoseEstimation.updateEstimatedPose(pose.getEstimatedPosition(), this);
	}

	public void updateVision() {
            if (LimelightHelpers.getFiducialID(VisionConstants.LimelightName) < 0) {
                return;
            }

			LimelightHelpers.SetRobotOrientation(VisionConstants.LimelightName, getYaw().getDegrees(), 0, 0, 0, 0, 0);
            double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.LimelightName)
                    .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
            double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);

			LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightName);

			if (mt2.tagCount < 1 || Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
				return;
			}

            Pose2d poseFromVision = new Pose2d(mt2.pose.getTranslation(), getYaw());

            double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                    - (LimelightHelpers.getLatency_Capture(VisionConstants.LimelightName)
                            + LimelightHelpers.getLatency_Pipeline(VisionConstants.LimelightName)) / 1000;

            pose.addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));
	}

	@Override
	public void simulationPeriodic() {
		/* Assume 20ms update rate, get battery voltage from WPILib */
		swerve.updateSimState(0.020, RobotController.getBatteryVoltage());
	}

	public void initlogs() {
		logged_field = new LoggedField("PoseEstimation", logger, SwerveLogging.Pose, true);
		logged_modules = new LoggedSweveModules("modules", logger, this, SwerveLogging.Modules);

		logged_field.addPose2d("PoseEstimation", () -> PoseEstimation.getEstimatedPose(), true);
		logged_field.addPose2d("Closest Reef", this::getClosestReef, true);
		logged_field.addPose2d("Selected Reef", this::getSelectedReef, true);
		logged_field.addPose2d("Closest Barge", this::getClosestBarge, true);
		logged_field.addPose2d("limelight pose", () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose, true);
		logger.add(logged_field);

		logger.add(logged_modules);
	}

	public void putSwerveState() {
		SmartDashboard.putData("Swerve Drive", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle", () -> getModuleAngle(0).getDegrees(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> getModuleSpeed(0), null);

				builder.addDoubleProperty("Front Left Angle", () -> getModuleAngle(0).getDegrees(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> getModuleSpeed(0), null);

				builder.addDoubleProperty("Front Right Angle", () -> getModuleAngle(1).getDegrees(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> getModuleSpeed(1), null);

				builder.addDoubleProperty("Back Left Angle", () -> getModuleAngle(2).getDegrees(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> getModuleSpeed(2), null);

				builder.addDoubleProperty("Back Right Angle", () -> getModuleAngle(3).getDegrees(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> getModuleSpeed(3), null);

				builder.addDoubleProperty("Robot Angle", () -> getYaw().getDegrees(), null);
			}
		});
	}
}
