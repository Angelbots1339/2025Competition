// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
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
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LoggingConstants.SwerveLogging;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.swerve;
	SwerveDrivePoseEstimator pose = new SwerveDrivePoseEstimator(swerve.getKinematics(), getRelativeYaw(),
			swerve.getState().ModulePositions, Pose2d.kZero);

	private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
			.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.RobotCentric m_robotRequest = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private PIDController angularDrivePID = new PIDController(SwerveConstants.angularDriveKP,
			SwerveConstants.angularDriveKI, SwerveConstants.angularDriveKD);

	private PIDController pidToPoseXController = new PIDController(SwerveConstants.pidToPoseKP, 0,
			SwerveConstants.pidToPoseKD);
	private PIDController pidToPoseYController = new PIDController(SwerveConstants.pidToPoseKP, 0,
			SwerveConstants.pidToPoseKD);

	private LoggedSubsystem logger = new LoggedSubsystem("Swerve");
	private LoggedSweveModules logged_modules;
	private LoggedField logged_field;

	public boolean use_vision = false;

	/** Creates a new Swerve. */
	public Swerve() {
		configPathPlanner();
		angularDrivePID.setTolerance(SwerveConstants.angularDriveTolerance);
		angularDrivePID.enableContinuousInput(0, 360);
		pidToPoseXController.setTolerance(SwerveConstants.pidToPoseTolerance);
		pidToPoseYController.setTolerance(SwerveConstants.pidToPoseTolerance);

		initlogs();
		// putSwerveState();
	}
	public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn,
			Supplier<Boolean> fieldRelative) {
		return drive(x, y, turn, fieldRelative, () -> false);
	}

	public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn,
			Supplier<Boolean> fieldRelative,Supplier<Boolean> slowdown) {
		return run(() -> {
			ChassisSpeeds speeds = new ChassisSpeeds(x.get(),
					y.get(), turn.get()).times(slowdown.get() ? SwerveConstants.slowedSpeed / SwerveConstants.maxspeed: 1);
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

	/**
     *
     * @param translationX  Meters/second
     * @param translationY  Meters/second
     * @param rotation      Rad/second
     * @param fieldOriented Use field oriented drive?
     * @param skewReduction Use Skew Reduction?
     * @return Drive Command
     */
    public Command angularDrive(Supplier<Double> translationX, Supplier<Double> translationY,
            Supplier<Rotation2d> desiredRotation,
            Supplier<Boolean> fieldOriented) {

        return run(() -> {

            ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);
            SwerveRequest req;

            if (fieldOriented.get()) {
                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());

                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                        .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
                        .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

            } else {
                req = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
                                : DriveRequestType.Velocity)
                        .withVelocityX(translationX.get())
                        .withVelocityY(translationY.get())
                        .withRotationalRate(speeds.omegaRadiansPerSecond);
            }

            swerve.setControl(req);
        });
    }


    public void pidToPose(Pose2d target) {
		pidToPose(target, 0);
	}
    public void pidToPose(Pose2d target, double maxspeed) {
		if (maxspeed == 0) {
			maxspeed = SwerveConstants.pidToPoseMaxSpeed;
		}
        double x = -MathUtil.clamp(
                pidToPoseXController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getX(),
                        target.getX())
                        + Math.signum(pidToPoseXController.getError()) * Math.abs(SwerveConstants.pidToPoseKS),
                -maxspeed, maxspeed);
        double y = -MathUtil.clamp(
                pidToPoseYController.calculate(PoseEstimation.getEstimatedPose().getTranslation().getY(),
                        target.getY())
                        + Math.signum(pidToPoseYController.getError()) * Math.abs(SwerveConstants.pidToPoseKS),
                -maxspeed, maxspeed);

        SmartDashboard.putNumber("PidXError", pidToPoseXController.getPositionError());
        SmartDashboard.putNumber("PidYError", pidToPoseYController.getPositionError());

        angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : x,
                () -> pidToPoseYController.atSetpoint() ? 0 : y, () -> target.getRotation());
	}


	public void angularDriveRequest(Supplier<Double> translationX, Supplier<Double> translationY,
			Supplier<Rotation2d> desiredRotation) {

		ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

		SwerveRequest req;

		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());

		req = new SwerveRequest.FieldCentric()
				.withDriveRequestType(DriverConstants.openLoopDrive ? DriveRequestType.OpenLoopVoltage
						: DriveRequestType.Velocity)
				.withVelocityX(-speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
				.withVelocityY(-speeds.vyMetersPerSecond) // Drive left with negative X (left)
				.withRotationalRate(speeds.omegaRadiansPerSecond);

		swerve.setControl(req);
	}

	public boolean isAngularDriveAtSetpoint() {
        return angularDrivePID.atSetpoint();
	}

	public boolean isAtPose() {
        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && isAngularDriveAtSetpoint();
    }

	public void resetPose(Pose2d pose) {
		setYaw(pose.getRotation());
		PoseEstimation.updateEstimatedPose(pose, this);
		swerve.resetPose(pose);
		this.pose.resetPose(pose);
		updatePose();
		use_vision = true;
	}

	private ChassisSpeeds angularPIDCalc(Supplier<Double> translationX, Supplier<Double> translationY,
			Supplier<Rotation2d> desiredRotation) {
		double pid = angularDrivePID.calculate(getYaw().getDegrees(), desiredRotation.get().getDegrees());

		ChassisSpeeds speeds = new ChassisSpeeds(translationX.get(), translationY.get(),
				MathUtil.clamp(
						angularDrivePID.atSetpoint() ? 0 : pid + (SwerveConstants.angularDriveKS * Math.signum(pid)),
						-SwerveConstants.maxturn, SwerveConstants.maxturn));

		return speeds;
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
					return false;
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
		swerve.getPigeon2().setYaw(angle.getDegrees(), 0.1);
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
		updatePose();
	}

	public void updatePose() {
		if (use_vision) {
			updateVision(false);
		}
		pose.update(getYaw(), swerve.getState().ModulePositions);
		PoseEstimation.updateEstimatedPose(pose.getEstimatedPosition(), this);
	}

	public void addVision(String limelightname, boolean trust) {
		LimelightHelpers.SetRobotOrientation(limelightname, getYaw().getDegrees(), 0, 0, 0, 0, 0);
		double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(limelightname)
				.getTranslation().getNorm(); // Find direct distance to target for std dev calculation
		double std = VisionConstants.calcStdDev(tagDistance);
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

		if (mt2.avgTagDist < 4)
			pose.addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(std, std, std));
	}

	public void updateVision(boolean trust) {
		if (Robot.isReal()) {
		addVision(VisionConstants.LimelightRightName, trust);
		addVision(VisionConstants.LimelightLeftName, trust);
		}
	}

	@Override
	public void simulationPeriodic() {
		/* Assume 20ms update rate, get battery voltage from WPILib */
		swerve.updateSimState(0.020, RobotController.getBatteryVoltage());
	}

	public void initlogs() {
		logged_field = new LoggedField("PoseEstimation", logger, SwerveLogging.Pose, true);
		logged_modules = new LoggedSweveModules("modules", logger, this, SwerveLogging.Modules);

		logged_field.addPose2d("PoseEstimation", () -> PoseEstimation.getEstimatedPose(), false);
		logged_field.addPose2d("Closest Reef", () -> AlignUtil.offsetPose(AlignUtil.getClosestReef(), AlignUtil.coralOffset), false);
		// logged_field.addPose2d("Selected Reef", () -> AlignUtil.getSelectedReef(), true);
		// logged_field.addPose2d("Closest Barge", () -> AlignUtil.getClosestBarge(), true);
		// logged_field.addPose2d("Limelight Left", () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightLeftName).pose != null ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightLeftName).pose : Pose2d.kZero, true);
		// logged_field.addPose2d("Limelight Right", () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightRightName).pose != null ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LimelightRightName).pose : Pose2d.kZero, true);
		// logger.addDouble("target angle error", () -> angularDrivePID.getError(), SwerveLogging.PidPose);
		// logger.addDouble("target angle", () -> angularDrivePID.getSetpoint(), SwerveLogging.PidPose);
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
