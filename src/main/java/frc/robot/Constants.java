package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

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

	public class DriverConstants {
		public static final int driverPort = 0;
		public static final int operatorPort = 1;
		public static final double joystickDeadband = 0.2;

		public static double deadbandJoystickValues(double val, double max) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), 1),
					joystickDeadband) * max * Math.signum(val);
		}
	}
}
