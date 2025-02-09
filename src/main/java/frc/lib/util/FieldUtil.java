package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
	/* all locations are relative */
	public static final Pose2d BlueProcessorOpening = new Pose2d(Units.inchesToMeters(235.73),
		Units.inchesToMeters(-0.15), Rotation2d.fromDegrees(90));
	public static final Pose2d RedProcessorOpening = new Pose2d(Units.inchesToMeters(455.15),
		Units.inchesToMeters(317.15), Rotation2d.fromDegrees(270));

	public static final Pose2d BlueLeftCoralStation = new Pose2d(Units.inchesToMeters(33.51),
		Units.inchesToMeters(291.20), Rotation2d.fromDegrees(306));
	public static final Pose2d BlueRightCoralStation = new Pose2d(Units.inchesToMeters(33.51),
		Units.inchesToMeters(25.80), Rotation2d.fromDegrees(54));

	public static final Pose2d RedRightCoralStation = new Pose2d(Units.inchesToMeters(657.37),
		Units.inchesToMeters(291.20), Rotation2d.fromDegrees(234));
	public static final Pose2d RedLeftCoralStation = new Pose2d(Units.inchesToMeters(657.37),
		Units.inchesToMeters(25.80), Rotation2d.fromDegrees(126));

	/* center of our barge and edge of the net we can score in from our side */
	public static final Pose2d AllianceSideBlueBargeCenter = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Rotation2d.fromDegrees(180));
	public static final Pose2d OpponateSideBlueBargeCenter = new Pose2d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), Rotation2d.fromDegrees(0));

	public static final Pose2d AllianceSideRedBargeCenter = new Pose2d(Units.inchesToMeters(365.20), Units.inchesToMeters(75.39), Rotation2d.fromDegrees(0));
	public static final Pose2d OpponateSideRedBargeCenter = new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Rotation2d.fromDegrees(180));
	public static final double BargeWidth = Units.inchesToMeters(146.50);

	/* reef locations starting from farthest reef and going counter clockwise
	 * a "reef" is a set of two "prongs" where coral can be scored.  there are 6 sets of these
	 * in a hexagonal pattern
	*/
	public static final Pose2d BlueReefPoses[] = {
		new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)),
		new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
		new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
		new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)),
		new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
		new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)),
	};

	public static final Pose2d RedReefPoses[] = {
		new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)),
		new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
		new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)),
		new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)),
		new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
		new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
	};

	public static boolean isRedAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (alliance.isEmpty())
			return false;

		if (alliance.get() == Alliance.Red) {
			return true;
		}

		return false;
	}

	public static Pose2d[] getReef() {
		if (isRedAlliance())
			return RedReefPoses;

		return BlueReefPoses;
	}

	public static Pose2d getProcessor() {
		if (isRedAlliance())
			return RedProcessorOpening;

		return BlueProcessorOpening;
	}

	public static Pose2d getRightCoralStation() {
		if (isRedAlliance())
			return RedRightCoralStation;

		return BlueRightCoralStation;
	}

	public static Pose2d getLeftCoralStation() {
		if (isRedAlliance())
			return RedLeftCoralStation;

		return BlueLeftCoralStation;
	}

	public static Pose2d getAllianceSideBargeCenter() {
		if (isRedAlliance())
			return AllianceSideRedBargeCenter;

		return AllianceSideBlueBargeCenter;
	}

	public static Pose2d getOpponateSideBargeCenter() {
		if (isRedAlliance())
			return OpponateSideRedBargeCenter;

		return OpponateSideBlueBargeCenter;
	}
}
