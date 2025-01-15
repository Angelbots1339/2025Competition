package frc.robot;

import edu.wpi.first.math.MathUtil;

public class Constants {
	public class SwerveConstants {
		public static final double maxspeed = 5;
		public static final double maxturn = 2 * Math.PI;
	}
	public class DriverConstants {
		public static final double joystickDeadband = 0.1;

		public static double deadbandJoystickValues(double val, double max) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), 1),
					joystickDeadband) * max * Math.signum(val);
		}
	}
}
