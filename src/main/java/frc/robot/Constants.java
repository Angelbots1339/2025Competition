package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
		public static final Angle angleErrorTolerence = Degrees.of(1);

		public static final double angleMotorRatio = 9 * 32.0/14.0;
		public static final Angle angleMotorOffset = Rotations.of(-0.75);

		public static final Angle maxAngle = Degrees.of(90);
		public static final Angle minAngle = Degrees.of(0);
		public static final Angle intakeAngle = Degrees.of(13);
		public static final Angle startingAngle = Degrees.of(90);
		/* TODO: find what this angle is */
		public static final Angle algaeStayAngle = Degrees.of(45);


		public static final Voltage intakeVolts = Volts.of(4.0);

		public static final TalonFXConfiguration wheelConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.CounterClockwise_Positive)
			);

		public static final SlotConfigs pid = new SlotConfigs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
			/* volts per rotation */
			.withKP(30)
			.withKI(0)
			.withKD(0)
			.withKG(0.1)
			.withKS(0.1);

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
			.withMotionMagic(
					new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(DegreesPerSecond.of(45 * 144))
						.withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(45 * 144))
			)
			.withFeedback(feedback)
			.withSoftwareLimitSwitch(
				new SoftwareLimitSwitchConfigs()
					.withForwardSoftLimitEnable(true)
					.withForwardSoftLimitThreshold(maxAngle)
					.withReverseSoftLimitEnable(true)
					.withReverseSoftLimitThreshold(minAngle)
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
		/* all units are in meters */
		public static final double BaseHeight = Units.inchesToMeters(40);
		public static final double StageHeight = Units.inchesToMeters(49);

		public static final int LeaderPort = 4;
		public static final int FollowerPort = 5;

		public static final double GearRatio = 9;
		private static final double Radius = Units.inchesToMeters(0.6589);
		public static final double ErrorTolerence = 0.01;

		/* heights are in meters */
		public class Heights {
			public static final double Max = 0.57; // meters
			public static final double Min = Units.inchesToMeters(0);
		}

		/* plot voltage and speed */
		public static final Slot0Configs pid = new Slot0Configs()
				.withKP(5)
				.withKI(0)
				.withKD(0)
				.withGravityType(GravityTypeValue.Elevator_Static)
				.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
				.withKG(0.245)
				.withKV(1)
				.withKA(0.04)
				.withKS(0);

		public static final SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(true)
				.withReverseSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(metersToRotations(Heights.Max))
				.withReverseSoftLimitThreshold(metersToRotations(Heights.Min));

		public static final MotionMagicConfigs motionmagic = new MotionMagicConfigs()
			.withMotionMagicAcceleration(metersToRotations(1.8))
			.withMotionMagicCruiseVelocity(metersToRotations(1));

		public static final TalonFXConfiguration baseConfig = new TalonFXConfiguration()
				.withMotorOutput(
					new MotorOutputConfigs()
					.withInverted(InvertedValue.CounterClockwise_Positive)
					.withNeutralMode(NeutralModeValue.Brake)
				)
				.withFeedback(
					new FeedbackConfigs()
						.withSensorToMechanismRatio(GearRatio)
				)
				.withCurrentLimits(
					new CurrentLimitsConfigs()
						.withStatorCurrentLimit(40)
				)
				.withSlot0(pid)
				.withMotionMagic(motionmagic);

		public static final TalonFXConfiguration leaderConfigs = baseConfig.withSoftwareLimitSwitch(limits);

		public static final MotionMagicVoltage PositionRequest = new MotionMagicVoltage(0);

		public static final double rotationToMeters(double rotations) {
			return rotations * 2 * Math.PI * Radius;
		}

		public static final double metersToRotations(double meters) {
			return meters / (2.0 * Math.PI * Radius);
		}
    }

	public class EndEffectorConstants {
		public static final int anglePort = 6;
		public static final int wheelPort = 7;
		public static final int encoderPort = 0;
		public static final int sensorPort = 1;

		public static final double gearRatio = 32.0 / 16.0;
		/* TODO: find the actual angles for these
		 * first find the 90 position by setting the end effector where it won't tip
		 * then set the offset in talonfx with (current rotation found by the cancoder) - 0.25
		 */
		public static final Angle maxAngle = Degrees.of(120);
		public static final Angle minAngle = Degrees.of(0);

		public static final Angle defaultAngle = Degrees.of(110);
		public static final Angle intakeAngle = Degrees.of(50);
		public static final Angle algaeAngle = Degrees.of(49);

		public static final Angle angleErrorTolerence = Degrees.of(1);
		public static final double hasAlgaeThreshold = 250; /* mm */

		public static final Voltage intakeVolts = Volts.of(3);
		public static final Voltage algaeHoldVoltage = Volts.of(2);

		public static final SlotConfigs pid = new SlotConfigs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
			.withKP(0)
			.withKI(0)
			.withKD(0)
			.withKS(0)
			.withKG(0);

		public static final MotionMagicConfigs motion = new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(DegreesPerSecond.of(12.5))
			// .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(12.5));
			;

		public static final TalonFXConfiguration baseAngleConfig = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Brake)
					.withInverted(InvertedValue.Clockwise_Positive)
			)
			.withSoftwareLimitSwitch(
				new SoftwareLimitSwitchConfigs()
					.withForwardSoftLimitEnable(true)
					.withReverseSoftLimitEnable(true)
					.withForwardSoftLimitThreshold(maxAngle)
					.withReverseSoftLimitThreshold(minAngle)
			)
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimitEnable(true)
					.withStatorCurrentLimit(40)
			)
			.withFeedback(
				new FeedbackConfigs()
					/* might need to invert in tunerfx */
					.withRemoteCANcoder(new CANcoder(encoderPort))
					.withSensorToMechanismRatio(gearRatio)
			);

		public static final TalonFXConfiguration angleConfig = baseAngleConfig
			.withMotionMagic(
				new MotionMagicConfigs()
			)
			.withSlot0(Slot0Configs.from(pid));

		public static final TalonFXConfiguration wheelConfig = new TalonFXConfiguration()
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(40)
			)
			.withMotorOutput(
				new MotorOutputConfigs()
				.withNeutralMode(NeutralModeValue.Brake)
				.withInverted(InvertedValue.Clockwise_Positive)
			);
	}

	public class SequencingConstants {
		public static final double IntakeHitPoint = 0.14;
		public static final double IntakeHitPointBound = 0.05;

		/* TODO: find the angle at which the end effector will not hit the intake with no algae */
		public static final Angle intakeAvoidAngle = Degrees.of(45);
		/* TODO: find the angle at which the intake is no longer in the way of the algae + end effector */
		public static final Angle algaeAvoidAngle = Degrees.of(40);

		/* TODO: find the angle at which the end effector will not hit the elevator when moving upwards */
		public static final Angle endEffectorAvoidAngle = Degrees.of(30);
		/* TODO: find the angle at which we will start to score algae */
		public static final Angle endEffectorBargeAngle = Degrees.of(100);


		public static enum Heights {
			Intake(0),
			Home(0),
			A1(0.2),
			A2(0.34),
			Barge(ElevatorConstants.Heights.Max);

			public final double height;

			Heights(double height) {
				this.height = height;
			}
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
			Elevator,
			EndEffector,
			None,
		}
	}
}
