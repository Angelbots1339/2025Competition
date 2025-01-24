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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldUtil;
import frc.lib.util.PoseEstimation;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedSweveModules;
import frc.robot.LoggingConstants.SwerveLogging;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.swerve;

	private double maxspeed = 3;
	private double maxturn = Math.PI * 2;

	private double coralScoreOffsetY = Units.inchesToMeters(0);
	// private double coralScoreOffsetX = Units.inchesToMeters(32.75);
	private double coralScoreOffsetX = 0.940;

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

	/** Creates a new Swerve. */
	public Swerve() {
		configPathPlanner();
		swerve.getPigeon2().setYaw(0);

		initlogs();
		putSwerveState();
	}

	public void drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn, boolean fieldRelative) {
		ChassisSpeeds speeds = new ChassisSpeeds(x.get() * maxspeed, y.get() * maxspeed, turn.get() * maxturn);
		SwerveRequest req;

		if (fieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());
		}

		req = m_robotRequest
			.withVelocityX(speeds.vxMetersPerSecond)
			.withVelocityY(speeds.vyMetersPerSecond)
			.withRotationalRate(speeds.omegaRadiansPerSecond);

		swerve.setControl(req);
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
					// 	return alliance.get() == DriverStation.Alliance.Red;
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
		this.selectedReef = FieldUtil.getReef()[i];
	}

	public Pose2d getSelectedReef() {
		return selectedReef;
	}

	public Command driveToPose(Pose2d target) {
		PathConstraints constraints = new PathConstraints( 3.0, 4.0,
				maxturn, Units.degreesToRadians(720));
		return AutoBuilder.pathfindToPose(target, constraints, 0.0);
	}

	public Command driveToClosestReef() {
		return driveToPose(getReefScoreSpot(getClosestReef()));
	}

	public Command driveToSelectedReef() {
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
		Pose2d target = PoseEstimation.getEstimatedPose().nearest(List.of(FieldUtil.getAllianceSideBargeCenter(), FieldUtil.getOpponateSideBargeCenter()));
		return target.plus(new Transform2d(bargeOffset, Rotation2d.kZero));
	}

	public Command driveToClosestBarge() {

		return driveToPose(getClosestBarge());
	}

	@Override
	public void periodic() {
		PoseEstimation.updateEstimatedPose(swerve.getState().Pose, this);
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
