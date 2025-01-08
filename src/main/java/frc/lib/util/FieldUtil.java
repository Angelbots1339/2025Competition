package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
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