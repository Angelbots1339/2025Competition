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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PoseEstimation;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.Swerve;
	private final Field2d m_field = new Field2d();

	private double maxspeed = 3;
	private double maxturn = Math.PI * 2;

	private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(DriveRequestType.Velocity);
	private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
			.withDeadband(maxspeed * 0.1).withRotationalDeadband(Math.PI / 2 * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	/** Creates a new Swerve. */
	public Swerve() {
		configPathPlanner();
		swerve.getPigeon2().setYaw(0);

		putSwerveState();
		SmartDashboard.putData("Field", m_field);
	}

	public void fieldCentricDrive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
		SwerveRequest req = m_driveRequest
				.withVelocityX(x.get() * maxspeed)
				.withVelocityY(y.get() * maxspeed)
				.withRotationalRate(turn.get() * maxturn)
				.withDeadband(0.2 * maxspeed)
				.withRotationalDeadband(0.2 * maxturn);

		swerve.setControl(req);
	}

	void resetPose(Pose2d pose) {
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
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	public Rotation2d getYaw() {
		return swerve.getPigeon2().getRotation2d();
	}

	public void setYaw(Rotation2d angle) {
		swerve.getPigeon2().setYaw(angle.getDegrees());
	}

	public Rotation2d getModuleAngle(int i) {
		return swerve.getModule(i).getCurrentState().angle;
	}

	public double getModuleSpeed(int i) {
		return swerve.getModule(i).getCurrentState().speedMetersPerSecond;
	}

	@Override
	public void periodic() {
		PoseEstimation.updateEstimatedPose(swerve.getState().Pose, this);

		m_field.setRobotPose(PoseEstimation.getEstimatedPose());
	}


	@Override
	public void simulationPeriodic() {
		/* Assume 20ms update rate, get battery voltage from WPILib */
		swerve.updateSimState(0.020, RobotController.getBatteryVoltage());
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
