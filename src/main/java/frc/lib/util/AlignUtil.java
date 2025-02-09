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
	/* offset is relative to robot */
	private static Translation2d coralOffset = new Translation2d(RobotConstants.length / 2, 0);
	private static Translation2d processorOffset = new Translation2d(RobotConstants.length / 2, 0);
	private static Translation2d bargeOffset = new Translation2d(Units.inchesToMeters(-24 / 2.0), 0.0);

	private static int selectedReefindex = -1;
	private static Pose2d selectedReef = new Pose2d(0, 0, Rotation2d.kZero);


	public Command driveToPose(Pose2d target) {
		PathConstraints constraints = new PathConstraints(SwerveConstants.maxspeed, 4.0,
				SwerveConstants.maxturn, Units.degreesToRadians(720));
		return AutoBuilder.pathfindToPose(target, constraints, 0.0);
	}

	public Command driveToClosestReef() {
		return driveToPose(getReefScoreSpot(getClosestReef()));
	}

	public Command driveToSelectedReef() {
		return driveToSelectedReef(selectedReefindex);
	}

	public Command driveToSelectedReef(int i) {
		if (i < 0 || i > 5)
			return Commands.none();
		selectReef(i);
		if (selectedReef.equals(new Pose2d(0, 0, Rotation2d.kZero)))
			return Commands.none();

		return driveToPose(getReefScoreSpot(selectedReef));
	}

	public Command driveToLeftCoralStation() {
		return driveToPose(FieldUtil.getLeftCoralStation());
	}

	public Command driveToRightCoralStation() {
		return driveToPose(FieldUtil.getRightCoralStation());
	}

	public Command driveToClosestCoralStation() {
		return driveToPose(getClosestCoralStations());
	}

	public Command driveToClosestBarge() {
		return driveToPose(getClosestBarge());
	}

	public Command driveToProcessor() {
		Translation2d tmp = processorOffset;
		tmp.rotateBy(FieldUtil.getProcessor().getRotation());

		Pose2d processorScorePos = FieldUtil.getProcessor()
				.plus(new Transform2d(tmp.getX(), tmp.getY(), Rotation2d.k180deg));

		return driveToPose(processorScorePos);
	}

	public static Pose2d getClosestBarge() {
		Pose2d target = PoseEstimation.getEstimatedPose()
				.nearest(List.of(FieldUtil.getAllianceSideBargeCenter(), FieldUtil.getOpponateSideBargeCenter()));
		return target.plus(new Transform2d(bargeOffset, Rotation2d.kZero));
	}

	public static Pose2d getClosestCoralStations() {
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

	public Pose2d getReefScoreSpot(Pose2d reef) {
		Translation2d tmp = coralOffset;
		tmp.rotateBy(reef.getRotation());

		reef = reef.plus(new Transform2d(tmp.getX(), tmp.getY(), Rotation2d.k180deg));
		return reef;
	}

	public static void selectReef(int i) {
		selectedReef = FieldUtil.getReef()[i];
		selectedReefindex = i;
	}

	public static Pose2d getSelectedReef() {
		return selectedReef;
	}
}
