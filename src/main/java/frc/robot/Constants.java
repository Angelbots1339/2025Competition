package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
	public class SwerveConstants {
		public static final double maxspeed = 5;
		public static final double maxturn = 2 * Math.PI;


		public static final PathConstraints PathPlannerConstraints = new PathConstraints( 3.0, 4.0,
				maxturn, Units.degreesToRadians(720));
	}

	public class RobotConstants {
		public static final double length = 0.938;
		public static final double width = 0.8;
	}

	public class IntakeConstants {
		public static final int angleMotorPort = 13;
		public static final int angleMotorFollowerPort = 14;
		public static final int wheelMotorPort = 15;

		public static final double angleMotorRatio = 2.60225;
		public static final double angleMotorOffset = 0;
		public static final double angleFollowerMotorOffset = 0;

		public static final Angle insideAngle = Degrees.of(90);
		public static final Angle outsideAngle = Degrees.of(0);

		public static final Voltage intakeVolts = Volts.of(1.0);

		public static final TalonFXConfiguration wheelConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.CounterClockwise_Positive)
			);

		public static final SlotConfigs pid = new SlotConfigs()
					.withGravityType(GravityTypeValue.Arm_Cosine)
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
					.withKP(0)
					.withKI(0)
					.withKD(0)
					.withKS(0)
					.withKG(0);

		public static final TalonFXConfiguration angleConfigs = new TalonFXConfiguration()
			.withMotorOutput(
				new MotorOutputConfigs()
					.withInverted(InvertedValue.Clockwise_Positive)
			)
			.withSlot0(
				Slot0Configs.from(pid)
			)
			.withFeedback(
				new FeedbackConfigs()
					.withSensorToMechanismRatio(angleMotorRatio)
					.withFeedbackRotorOffset(angleMotorOffset)
			)
			.withSoftwareLimitSwitch(
				new SoftwareLimitSwitchConfigs()
					/* uses 0 as the fully out position and 90 as the completely in / vertical position */
					.withForwardSoftLimitEnable(true)
					.withForwardSoftLimitThreshold(insideAngle)
					.withReverseSoftLimitEnable(true)
					.withReverseSoftLimitThreshold(outsideAngle)
			);

		public static final FeedbackConfigs angleFollwerConfiguration = angleConfigs.Feedback.withFeedbackRotorOffset(angleFollowerMotorOffset);
	}

	public class DriverConstants {
		public static final int driverPort = 0;
		public static final int operatorPort = 1;
		public static final int testPort = 2;
		public static final double joystickDeadband = 0.2;

		public static double deadbandJoystickValues(double val, double max) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), 1),
					joystickDeadband) * max * Math.signum(val);
		}
	}

	public class TuningConstants {
		public enum TuningSystem {
			Intake,
			None,
		}
	}
}
