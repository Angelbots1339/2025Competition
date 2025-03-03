// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public final class LoggingConstants {
        // Shuffleboard automatically logs to both Onboard and Network Tables
        // Make sure you call everything in Robot.java that needs to be called
        public class SwerveLogging {
                public static LoggingLevel Modules = LoggingLevel.NONE;
                public static LoggingLevel Motors = LoggingLevel.NONE;
                public static LoggingLevel Gyro = LoggingLevel.NONE;
                public static LoggingLevel Pose = LoggingLevel.NONE;
                public static LoggingLevel PidPose = LoggingLevel.NONE;
                public static LoggingLevel Auto = LoggingLevel.NONE;
                public static LoggingLevel Main = LoggingLevel.NONE;
        }

		public class ElevatorLogging {
			public static LoggingLevel Leader = LoggingLevel.NONE;
			public static LoggingLevel Follower = LoggingLevel.NONE;
			public static LoggingLevel FollowerMotor = LoggingLevel.NONE;
			public static LoggingLevel LeaderMotor = LoggingLevel.NONE;
		}

		public class EndEffectorLogging {
			public static LoggingLevel Angle = LoggingLevel.NETWORK_TABLES;
			public static LoggingLevel Wheel = LoggingLevel.NONE;
			public static LoggingLevel TOF = LoggingLevel.NETWORK_TABLES;
		}

        public class RobotContainerLogging {
                public static LoggingLevel StickValues = LoggingLevel.NONE;
        }

        public class GlobalLoggingConstants {
                public static LoggingLevel Default = LoggingLevel.NONE;
        }

}
