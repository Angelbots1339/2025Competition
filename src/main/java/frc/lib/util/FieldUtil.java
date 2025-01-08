package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
	/* all locations are relative */
	public static final Pose2d ProcessorOpening = new Pose2d(Units.inchesToMeters(235.73),
		Units.inchesToMeters(-0.15), Rotation2d.fromDegrees(90));

	public static final Pose2d LeftCoralStation = new Pose2d(Units.inchesToMeters(33.51),
		Units.inchesToMeters(291.20), Rotation2d.fromDegrees(306));

	public static final Pose2d RightCoralStation = new Pose2d(Units.inchesToMeters(33.51),
		Units.inchesToMeters(25.80), Rotation2d.fromDegrees(54));

	public static final Pose2d FarReef = new Pose2d(Units.inchesToMeters(209.49),
		Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));

	public static final Pose2d NearReef = new Pose2d(Units.inchesToMeters(144.00),
		Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));

	/* counter clockwise
	 * a "reef" is a set of two "prongs" where coral can be scored.  there are 6 sets of these
	 * in a hexagonal pattern
	*/
	public static final Pose2d ReefPoses[] = {
		FarReef,
		new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(300)),
		new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(240)),
		NearReef,
		new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
		new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)),
	};

	boolean isRedAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (alliance.isEmpty())
			return false;

		if (alliance.get() == Alliance.Red) {
			return true;
		}

		return false;
	}
}
