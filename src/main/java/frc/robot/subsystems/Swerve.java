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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Swerve extends SubsystemBase {
	public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve = TunerConstants.Swerve;

	private double maxspeed = 1;
	private double maxturn = Math.PI * 2;

	private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    	.getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

	PIDController turnPid = new PIDController(0, 0, 0);

	private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
	   .withDeadband(maxspeed * 0.1).withRotationalDeadband(Math.PI / 2 * 0.1)
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);

	/** Creates a new Swerve. */
	public Swerve() {
		swerve.getPigeon2().setYaw(0);
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

	public Rotation2d getYaw() {
		return swerve.getPigeon2().getRotation2d();
	}

	public void setYaw(Rotation2d angle) {
		swerve.getPigeon2().setYaw(angle.getDegrees());
	}

	public Rotation2d getModuleAngle(int i) {
		return swerve.getModule(i).getCurrentState().angle;
	}

	@Override
	public void periodic() {
		publisher.set(swerve.getState().ModuleStates);
		SmartDashboard.putNumber("yaw", getYaw().getDegrees());
		SmartDashboard.putNumber("fl", getModuleAngle(0).getRotations());
		SmartDashboard.putNumber("fr", getModuleAngle(1).getRotations());
		SmartDashboard.putNumber("bl", getModuleAngle(2).getRotations());
		SmartDashboard.putNumber("br", getModuleAngle(3).getRotations());

		SmartDashboard.putNumber("br raw m/s", swerve.getModule(3).getDriveMotor().getVelocity().getValueAsDouble() * 4 * Math.PI);
	}
}
