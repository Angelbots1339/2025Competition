// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AlignUtil;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedSweveModules;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LoggingConstants.SwerveLogging;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.swerve;
	SwerveDrivePoseEstimator pose = new SwerveDrivePoseEstimator(swerve.getKinematics(), getRelativeYaw(), swerve.getState().ModulePositions, Pose2d.kZero);


	private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
			.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.RobotCentric m_robotRequest = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private LoggedSubsystem logger = new LoggedSubsystem("Swerve");
	private LoggedSweveModules logged_modules;
	private LoggedField logged_field;

	/** Creates a new Swerve. */
	public Swerve() {
		configPathPlanner();

		initlogs();
		//putSwerveState();
	}

	public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn,
			Supplier<Boolean> fieldRelative) {
		return run(() -> {
			ChassisSpeeds speeds = new ChassisSpeeds(x.get() * SwerveConstants.maxspeed, y.get() * SwerveConstants.maxspeed, turn.get() * SwerveConstants.maxturn);
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
		this.pose.resetPose(pose);
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

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
					}
					return FieldUtil.isRedAlliance() && !DriverStation.isTeleop();
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

	@Override
	public void periodic() {
			updateVision();
		pose.update(getYaw(), swerve.getState().ModulePositions);
		PoseEstimation.updateEstimatedPose(pose.getEstimatedPosition(), this);
	}

	public void addVision(String limelightname, double std) {
			LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightname);

			if (mt2 == null)
				return;

			if (mt2.tagCount < 1 || Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
				return;
			}

            Pose2d poseFromVision = new Pose2d(mt2.pose.getTranslation(), getYaw());

            double poseFromVisionTimestamp = Timer.getFPGATimestamp()
                    - (LimelightHelpers.getLatency_Capture(limelightname)
                            + LimelightHelpers.getLatency_Pipeline(limelightname)) / 1000;

            pose.addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(std, std, 0));
	}

	public void updateVision() {
			LimelightHelpers.SetRobotOrientation(VisionConstants.LimelightCenterName, getYaw().getDegrees(), 0, 0, 0, 0, 0);
            double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.LimelightCenterName)
                    .getTranslation().getNorm(); // Find direct distance to target for std dev calculation
            double xyStdDev2 = VisionConstants.calcStdDev(tagDistance);
			addVision(VisionConstants.LimelightCenterName, xyStdDev2);
			// addVision(VisionConstants.LimelightRightName, xyStdDev2);
			// addVision(VisionConstants.LimelightLeftName, xyStdDev2);
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
		logged_field.addPose2d("Closest Reef", () -> AlignUtil.getClosestReef(), true);
		logged_field.addPose2d("Selected Reef", () -> AlignUtil.getSelectedReef(), true);
		logged_field.addPose2d("Closest Barge", () -> AlignUtil.getClosestBarge(), true);
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
