// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector;

public class EndEffectorTuning extends Command {
	EndEffector endeffector;

	private static ShuffleboardTab tab = Shuffleboard.getTab("EndEffector Tuning");
	private static ShuffleboardLayout pid = tab.getLayout("PID", BuiltInLayouts.kList);
	private static ShuffleboardLayout motion = tab.getLayout("Motion Magic", BuiltInLayouts.kList);
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static Voltage volt_target = Volts.zero();
	private static Angle targetAngle = Degrees.zero();

	private static GenericEntry target = tab.add("target", EndEffectorConstants.minAngle.in(Degrees))
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", EndEffectorConstants.maxAngle.in(Degrees)))
			.getEntry();

	private static GenericEntry volts = tab.add("volts", 1)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static GenericEntry p = pid.add("P", EndEffectorConstants.pid.kP)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry i = pid.add("I", EndEffectorConstants.pid.kI)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry d = pid.add("D", EndEffectorConstants.pid.kD)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry g = pid.add("G", EndEffectorConstants.pid.kG)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry s = pid.add("S", EndEffectorConstants.pid.kS)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static GenericEntry cv = motion.add("Cruise Velocity", EndEffectorConstants.motion.MotionMagicCruiseVelocity)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry a = motion.add("Acceleration", EndEffectorConstants.motion.MotionMagicAcceleration)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();


	private static Trigger setAngle = new Trigger(() -> test.getBButton());
	private static Trigger angleUp = new Trigger(() -> test.getPOV() == 0);
	private static Trigger angleDown = new Trigger(() -> test.getPOV() == 180);
	private static Trigger intakeRun = new Trigger(() -> test.getAButton());

	public EndEffectorTuning(EndEffector endeffector) {
		this.endeffector = endeffector;
		addRequirements(endeffector);
	}

	@Override
	public void initialize() {
		setAngle.whileTrue(Commands.run(() -> endeffector.setAngle(() -> targetAngle)));
		angleUp.onTrue(Commands.runOnce(() -> target.setDouble(targetAngle.in(Degrees) + 5)));
		angleDown.onTrue(Commands.runOnce(() -> target.setDouble(targetAngle.in(Degrees) - 5)));

		intakeRun.whileTrue(Commands.run(() -> endeffector.runIntake(volt_target))).whileFalse(Commands.run(() -> endeffector.runIntake(Volts.zero())));
		endeffector.stop();
	}

	@Override
	public void execute() {
		targetAngle = Degrees.of(MathUtil.clamp(target.getDouble(0), EndEffectorConstants.minAngle.in(Degrees), EndEffectorConstants.maxAngle.in(Degrees)));

		volt_target = Volts.of(MathUtil.clamp(volts.getDouble(0), -12, 12));

		SlotConfigs tmp = EndEffectorConstants.pid
				.withKP(p.getDouble(0))
				.withKI(i.getDouble(0))
				.withKD(d.getDouble(0))
				.withKG(g.getDouble(0))
				.withKS(s.getDouble(0));

		endeffector.setPID(tmp);

		MotionMagicConfigs motion = EndEffectorConstants.motion
			.withMotionMagicCruiseVelocity(cv.getDouble(0))
			.withMotionMagicAcceleration(a.getDouble(0));
		endeffector.setMotion(motion);
	}

	@Override
	public void end(boolean interrupted) {
		setAngle.whileTrue(Commands.none());
		angleUp.onTrue(Commands.none());
		angleDown.onTrue(Commands.none());
		intakeRun.whileTrue(Commands.none());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
