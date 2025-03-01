package frc.lib.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class AlignUtil {
	/* relative to robot */
	private static Transform2d coralOffset = new Transform2d(-RobotConstants.frontLength, 0, Rotation2d.kZero);
	private static Transform2d processorOffset = new Transform2d(-RobotConstants.frontLength, 0, Rotation2d.kZero);
	private static Transform2d stationOffset = new Transform2d(-RobotConstants.backLength, 0, Rotation2d.k180deg);
	private static Transform2d bargeOffset = new Transform2d(-RobotConstants.frontLength, 0, Rotation2d.kZero);

	private static int selectedReefindex = -1;
	private static Pose2d selectedReef = new Pose2d(0, 0, Rotation2d.kZero);

	public static Command driveToPose(Pose2d target) {
		PathConstraints constraints = new PathConstraints(SwerveConstants.maxspeed, 4.0,
				SwerveConstants.maxturn, Units.degreesToRadians(720));
		return AutoBuilder.pathfindToPose(target, constraints, 0.0);
	}

	public static Command driveToClosestReef() {
		return driveToPose(offsetPose(getClosestReef(), coralOffset));
	}

	public static Command driveToSelectedReef() {
		return driveToSelectedReef(selectedReefindex);
	}

	public static Command driveToSelectedReef(int i) {
		if (i < 0 || i > 5)
			return Commands.none();
		selectReef(i);
		if (selectedReef.equals(new Pose2d(0, 0, Rotation2d.kZero)))
			return Commands.none();

		return driveToPose(offsetPose(selectedReef, coralOffset));
	}

	public static Command driveToLeftCoralStation() {
		return driveToPose(offsetPose(FieldUtil.getLeftCoralStation(), stationOffset));
	}

	public static Command driveToRightCoralStation() {
		return driveToPose(offsetPose(FieldUtil.getRightCoralStation(), stationOffset));
	}

	public static Command driveToClosestCoralStation() {
		return driveToPose(offsetPose(getClosestCoralStation(), stationOffset));
	}

	public static Command driveToClosestBarge() {
		return driveToPose(offsetPose(getClosestBarge(), bargeOffset));
	}

	public static Command driveToProcessor() {
		return driveToPose(offsetPose(FieldUtil.getProcessor(), processorOffset));
	}

	public static Pose2d getClosestBarge() {
		return PoseEstimation.getEstimatedPose()
				.nearest(List.of(FieldUtil.getAllianceSideBargeCenter(), FieldUtil.getOpponateSideBargeCenter()));
	}

	public static Pose2d getClosestCoralStation() {
		Pose2d target = PoseEstimation.getEstimatedPose()
				.nearest(List.of(FieldUtil.getLeftCoralStation(), FieldUtil.getRightCoralStation()));
		return target;
	}

	public static Pose2d getClosestReef() {
		double dist = 10000000;
		int close = 0;
		Pose2d[] reefs = FieldUtil.getReef();
		for (int i = 0; i < reefs.length; i++) {
			Pose2d reef = reefs[i];
			double distToReef = PoseEstimation.getEstimatedPose().getTranslation().getDistance(reef.getTranslation());
			if (distToReef < dist) {
				dist = distToReef;
				close = i;
			}
		}

		return reefs[close];
	}

	public static void selectReef(int i) {
		selectedReef = FieldUtil.getReef()[i];
		selectedReefindex = i;
	}

	public static Pose2d getSelectedReef() {
		return selectedReef;
	}

	public static Pose2d offsetPose(Pose2d target, Transform2d offset) {
		Translation2d tmp = offset.getTranslation();
		tmp.rotateBy(target.getRotation());

		target = target.plus(new Transform2d(tmp.getX(), tmp.getY(), offset.getRotation().rotateBy(Rotation2d.k180deg)).times(-1));
		return target;

	}
}
