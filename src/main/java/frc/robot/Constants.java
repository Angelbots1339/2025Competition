package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
	public class SwerveConstants {
		public static final double maxspeed = 4;
		public static final double maxturn = 2 * Math.PI;

		public static final double angularDriveKP = 0.075;
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0.005;
		public static final double angularDriveKS = 0.4; // radians per sec
		public static final double angularDriveTolerance = 1.5; // Degrees

		public static final double pidToPoseKP = 2.5;
		public static final double pidToPoseKD = 0;
		public static final double pidToPoseKS = 0.15;
		public static final double pidToPoseTolerance = 0.03; // Meters
		public static final double pidToPoseMaxSpeed = 1; // Meters per second

		public static final PathConstraints PathPlannerConstraints = new PathConstraints(maxspeed, 4.0,
				maxturn, Units.degreesToRadians(720));
	}

	public class RobotConstants {
		public static final double backLength = Units.inchesToMeters(15.970);
		public static final double frontLength = Units.inchesToMeters(16.280);
		public static final double length = Units.inchesToMeters(32.25);
		public static final double width = Units.inchesToMeters(32.5);
	}

	public class IntakeConstants {
		public static final int leftAngleMotorPort = 1;
		public static final int rightAngleMotorPort = 3;
		public static final int wheelMotorPort = 2;
		public static final Angle angleErrorTolerence = Degrees.of(2.0);

		public static final double angleMotorRatio = 9 * 32.0/14.0;
		public static final Angle angleMotorOffset = Rotations.of(-0.75);

		public static final Angle insideAngle = Degrees.of(90);
		public static final Angle outsideAngle = Degrees.of(0);
		public static final Angle startingAngle = Degrees.of(90);


		public static final Voltage intakeVolts = Volts.of(2.0);

		public static final TalonFXConfiguration wheelConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.CounterClockwise_Positive)
			);

		public static final SlotConfigs pid = new SlotConfigs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
			.withKP(8)
			.withKI(0)
			.withKD(0)
			.withKG(0.25)
			.withKS(0.13);

		public static final FeedbackConfigs feedback = new FeedbackConfigs()
			.withSensorToMechanismRatio(angleMotorRatio)
			.withFeedbackRotorOffset(angleMotorOffset);

		public static final TalonFXConfiguration baseAngleConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.Clockwise_Positive)
					.withNeutralMode(NeutralModeValue.Brake)
			);

		public static final TalonFXConfiguration angleConfigs = baseAngleConfigs
			.withSlot0(Slot0Configs.from(pid))
			.withFeedback(feedback)
			.withSoftwareLimitSwitch(
				new SoftwareLimitSwitchConfigs()
					.withForwardSoftLimitEnable(true)
					.withForwardSoftLimitThreshold(insideAngle)
					.withReverseSoftLimitEnable(true)
					.withReverseSoftLimitThreshold(outsideAngle)
			);
	}

	public class DriverConstants {
		public static final int driverPort = 0;
		public static final int operatorPort = 1;
		public static final int testPort = 2;
		public static final double joystickDeadband = 0.1;
		public static final boolean openLoopDrive = true;

		public static double deadbandJoystickValues(double val, double max) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), 1),
					joystickDeadband) * max * Math.signum(val);
		}
	}

    public static final class ElevatorConstants {
		// TODO: figure the motor polarity and gear ratio
		/* all units are in meters */
		public static final double BaseHeight = Units.inchesToMeters(40);
		public static final double StageHeight = Units.inchesToMeters(49);
		public static final int LeaderPort = 14;
		public static final int FollowerPort = 15;
		// TODO: change to pitch diameter
		private static final double Radius = 0.1;
		public static final double ErrorTolerence = 0.02; // 1 cm

		/* heights are in meters */
		public class Heights {
			/* TODO: Tune */
			public static final double Max = Units.inchesToMeters(12);
			public static final double Min = Units.inchesToMeters(0);
		}

		public static final Slot0Configs PID = new Slot0Configs()
				.withKP(0)
				.withKI(0)
				.withKD(0)
				.withGravityType(GravityTypeValue.Elevator_Static)
				.withKG(0)
				.withKS(0);

		public static final SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(true)
				.withReverseSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(metersToRotations(Heights.Max))
				.withReverseSoftLimitThreshold(metersToRotations(Heights.Min));

		public static final MotionMagicConfigs motionmagic = new MotionMagicConfigs()
			.withMotionMagicAcceleration(metersToRotations(0.01))
			.withMotionMagicCruiseVelocity(metersToRotations(0.1));

		public static final TalonFXConfiguration baseConfig = new TalonFXConfiguration()
				.withMotorOutput(
					new MotorOutputConfigs()
					.withInverted(InvertedValue.Clockwise_Positive)
					.withNeutralMode(NeutralModeValue.Brake)
				)
				.withSlot0(PID)
				.withMotionMagic(motionmagic);

		public static final TalonFXConfiguration leaderConfigs = baseConfig.withSoftwareLimitSwitch(limits);

		public static final PositionVoltage PositionRequest = new PositionVoltage(0).withSlot(0);

		public static final double rotationToMeters(double rotations) {
			return rotations * 2 * Math.PI * Radius;
		}

		public static final double metersToRotations(double meters) {
			return meters / (2.0 * Math.PI * Radius);
		}
    }

	public class VisionConstants {
		/* offset for new bot
		 * left:
		 * - pitch -45
		 * - roll: 0
		 * - yaw: 180
		 * - right: -0.293231
		 * - Forward: 0.032436
		 * - Up: 0.706252
		 *
		 * Right:
		 * - pitch 45
		 * - roll: 0
		 * - yaw: 0
		 * - right: 0.293231
		 * - Forward: 0.027740
		 * - Up: 0.851537
		 */
		public static final String LimelightLeftName = "limelight-left";
		public static final String LimelightRightName = "limelight-right";

		public static double calcStdDev(double metersFromTarget) {
			return 0.08 * Math.pow(metersFromTarget, 2);
		}
	}

	public class TuningConstants {
		public enum TuningSystem {
			Swerve,
			Intake,
			None,
		}
	}
}
