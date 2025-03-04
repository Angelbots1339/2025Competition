// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorTuning extends Command {
	private Elevator elevator;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Elevator Tuning");
	private static ShuffleboardLayout pid = tab.getLayout("PID", BuiltInLayouts.kList);
	private static ShuffleboardLayout motion = tab.getLayout("Motion Magic", BuiltInLayouts.kList);
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static double targetHeight = 0;
	private static double step = 0.05;

	private static GenericEntry resetElevator = tab.add("Reset", false)
		.withWidget(BuiltInWidgets.kToggleButton)
		.getEntry();

	private static GenericEntry target = tab.add("target", ElevatorConstants.pid.kS)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", ElevatorConstants.Heights.Min, "max", ElevatorConstants.Heights.Max))
			.getEntry();

	private static GenericEntry p = pid.add("P", ElevatorConstants.pid.kP)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry i = pid.add("I", ElevatorConstants.pid.kI)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry d = pid.add("D", ElevatorConstants.pid.kD)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry g = pid.add("G", ElevatorConstants.pid.kG)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry s = pid.add("S", ElevatorConstants.pid.kS)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry v = pid.add("Kv", ElevatorConstants.pid.kV)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry ka = pid.add("Ka", ElevatorConstants.pid.kA)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static GenericEntry cv = motion.add("Cruise Velocity", ElevatorConstants.rotationToMeters(ElevatorConstants.motionmagic.MotionMagicCruiseVelocity))
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();
	private static GenericEntry a = motion.add("Acceleration", ElevatorConstants.rotationToMeters(ElevatorConstants.motionmagic.MotionMagicAcceleration))
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static Trigger setHeight = new Trigger(() -> test.getBButton());
	private static Trigger heightUp = new Trigger(() -> test.getPOV() == 0);
	private static Trigger heightDown = new Trigger(() -> test.getPOV() == 180);

	public ElevatorTuning(Elevator elevator) {
		this.elevator = elevator;
	}

	@Override
	public void initialize() {
		setHeight.whileTrue(elevator.setHeightCommand(() -> targetHeight));
		heightUp.onTrue(Commands.runOnce(() -> target.setDouble(Math.min(target.getDouble(0) + step, ElevatorConstants.Heights.Max))));
		heightDown.onTrue(Commands.runOnce(() -> target.setDouble(Math.max(target.getDouble(0) - step, ElevatorConstants.Heights.Min))));

		elevator.stop();
	}

	@Override
	public void execute() {
		targetHeight = target.getDouble(0);

		if (resetElevator.getBoolean(false)) {
			elevator.reset();
			resetElevator.setBoolean(false);
		}

		Slot0Configs tmp = ElevatorConstants.pid
				.withKP(p.getDouble(0))
				.withKI(i.getDouble(0))
				.withKD(d.getDouble(0))
				.withKG(g.getDouble(0))
				.withKA(ka.getDouble(0))
				.withKV(v.getDouble(0))
				.withKS(s.getDouble(0));

		elevator.setPID(tmp);

		MotionMagicConfigs motion = ElevatorConstants.motionmagic
			.withMotionMagicCruiseVelocity(ElevatorConstants.metersToRotations(cv.getDouble(0)))
			.withMotionMagicAcceleration(ElevatorConstants.metersToRotations(a.getDouble(0)));
		elevator.setMotion(motion);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
