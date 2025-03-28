package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
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
		public static final double slowedSpeed = 0.5;

		public static final double angularDriveKP = 0.075; // 0.075
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0.005;
		public static final double angularDriveKS = 0.2; // radians per sec
		public static final double angularDriveTolerance = 3; // Degrees

		public static final double pidToPoseKP = 2.5; // was 2.5
		public static final double pidToPoseKD = 0;
		public static final double pidToPoseKS = 0.15;
		public static final double pidToPoseTolerance = 0.02; // Meters
		public static final double pidToPoseMaxSpeed = 2; // Meters per second

		public static final PathConstraints PathPlannerConstraints = new PathConstraints(maxspeed, 4.0,
				maxturn, Units.degreesToRadians(720));
	}

	public class RobotConstants {
		public static final double backLength = Units.inchesToMeters(15.970);
		public static final double frontLength = Units.inchesToMeters(16.280);
		public static final double length = Units.inchesToMeters(32.25);
		public static final double width = Units.inchesToMeters(32.5);
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
			public static final double Max = 0.5734; // meters
			public static final double Min = Units.inchesToMeters(0);
		}

		public static final Slot0Configs pid = new Slot0Configs()
				.withKP(15)
				.withKI(0)
				.withKD(0)
				.withGravityType(GravityTypeValue.Elevator_Static)
				.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
				.withKG(0.254)
				.withKV(1)
				.withKA(0.04)
				.withKS(0);

		public static final SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(true)
				.withReverseSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(metersToRotations(Heights.Max))
				.withReverseSoftLimitThreshold(metersToRotations(Heights.Min));

		public static final MotionMagicConfigs motionmagic = new MotionMagicConfigs()
			.withMotionMagicAcceleration(metersToRotations(1.6))
			.withMotionMagicCruiseVelocity(metersToRotations(2.5));

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
		public static final int sensorPort = 9;

		// public static final double encoderOffset = -(0.9 - 0.5); // kyle's dead code

		// "zero" has been set based on the required 0 deg mark for the PID loop
		// on 3/25/2025 this was changed from -.25 due to chain work
		// we lost the old setting but when EE touches the ground, "current angle" should be around -34 deg, resultingtrue encoder = 0.071
		// on 3/13/2025 the new EE was add, hard stop should be around 99 deg, hits bumpers around 8 degrees
		public static final double encoderOffset = .12;
		public static double timeBeforeEncoderReset = 1.5;

		public static final double gearRatio = 32.0 / 16.0;
		public static final double gearbox = 9;
		public static final Angle maxAngle = Degrees.of(99); // was 90
		public static final Angle minAngle = Degrees.of(0);  // was -43

		public static final Angle defaultAngle = Degrees.of(80);
		public static final Angle intakeAngle = Degrees.of(18); // was 8, this was too low for new EE + Bumpers
		public static final double outtakeTime = 0.5;
		public static final Angle processorAngle = Degrees.of(55);

		public static final Angle angleErrorTolerence = Degrees.of(3);
		public static final double hasAlgaeThreshold = 90; /* mm */

		public static final Voltage intakeVolts = Volts.of(7.5);
		public static final Voltage coralIntakeVolts = Volts.of(-5);
		public static final Voltage coralIntakeSetVolts = Volts.of(0.3);
		public static final Voltage outtakeVolts = Volts.of(-12);
		public static final Voltage coralOuttakeVolts = Volts.of(-6);
		public static final Voltage algaeHoldVoltage = Volts.of(0.7);

		// Old settings below - increasing P allowed the EE to correct on overshoots when w/o a game piece and hit set points appropriately when w/ algae
		// P = 30
		// I = 0
		// D = 0
		// S = .1
		// G = .65
		public static final SlotConfigs pid = new SlotConfigs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
			.withKP(40)
			.withKI(0)
			.withKD(0)
			.withKS(0.15)
			.withKG(0.65);

		public static final MotionMagicConfigs motion = new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(6)
			.withMotionMagicAcceleration(3);

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
					.withStatorCurrentLimit(80)
					.withSupplyCurrentLimit(40)
			)
			.withFeedback(
				new FeedbackConfigs()
					.withSensorToMechanismRatio(gearRatio * gearbox)
			);

		public static final TalonFXConfiguration angleConfig = baseAngleConfig
			.withMotionMagic(motion)
			.withSlot0(Slot0Configs.from(pid))
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(80)
					.withSupplyCurrentLimit(40)
			);

		public static final TalonFXConfiguration wheelConfig = new TalonFXConfiguration()
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(60)
			)
			.withMotorOutput(
				new MotorOutputConfigs()
				.withNeutralMode(NeutralModeValue.Brake)
				.withInverted(InvertedValue.CounterClockwise_Positive)
			)
			.withCurrentLimits(
				new CurrentLimitsConfigs()
					.withStatorCurrentLimit(25)
					.withSupplyCurrentLimit(40)
			);
	}

	public class SequencingConstants {
		public static final Angle endEffectorAvoidAngle = Degrees.of(82);
		/* TODO: find the angle at which we will start to score algae */
		public static final Angle endEffectorBargeAngle = Degrees.of(74);
		public static final Angle reefAlgaeAngle = Degrees.of(50); // elevator 0.33


		public static enum SetPoints {
			Intake(0),
			Home(0),
			A1(0.12, reefAlgaeAngle, EndEffectorConstants.intakeVolts),
			A2(0.24, reefAlgaeAngle, EndEffectorConstants.intakeVolts),
			/* TODO: tune coral heights */
			L1(0.21),
			L2(0.1, Degrees.of(55)),
			L3(0.22, Degrees.of(55)),
			L4(0.4, Degrees.of(60)),
			Barge(0.57, Degrees.of(74)), // 0.571 max
			DeAlgae(0.14, Degrees.of(0), Volts.of(10)); // used in case of algae being inside

			public final double height;
			public final Angle angle;
			public final Voltage volts;

			SetPoints(double height) {
				this.height = height;
				this.angle = null;
				this.volts = null;
			}

			SetPoints(double height, Angle angle) {
				this.height = height;
				this.angle = angle;
				this.volts = null;
			}

			SetPoints(double height, Angle angle, Voltage volts) {
				this.height = height;
				this.angle = angle;
				this.volts = null;
			}
		}
	}

	public class VisionConstants {
		/* offset for new bot
		 * left:
		 * - pitch -30
		 * - roll: 180
		 * - yaw: 180
		 * - right: -0.292965
		 * - Forward: 0.042503
		 * - Up: 0.705851
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
			Elevator,
			EndEffector,
			Superstructure,
			None,
		}
	}
}
