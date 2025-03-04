// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class SuperstructureTuning extends Command {
	private Elevator elevator;
	private EndEffector endeffector;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Superstructure Tuning");
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static double targetHeight = 0;
	private static Voltage volt_target = Volts.zero();
	private static Angle targetAngle = Degrees.zero();

	private static GenericEntry height = tab.add("target height", 0)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", ElevatorConstants.Heights.Min, "max", ElevatorConstants.Heights.Max))
			.getEntry();

	private static GenericEntry angle = tab.add("target angle", EndEffectorConstants.defaultAngle.in(Degrees))
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", EndEffectorConstants.minAngle.in(Degrees), "max", EndEffectorConstants.maxAngle.in(Degrees)))
			.getEntry();

	private static GenericEntry volts = tab.add("volts", EndEffectorConstants.intakeVolts.in(Volts))
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static Trigger setHeight = new Trigger(() -> test.getBButton());
	private static Trigger setAngle = new Trigger(() -> test.getYButton());
	private static Trigger intakeRun = new Trigger(() -> test.getAButton());

	public SuperstructureTuning(Elevator elevator, EndEffector endeffector) {
		this.elevator = elevator;
		this.endeffector = endeffector;
		addRequirements(elevator, endeffector);
	}

	public double clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}

	@Override
	public void initialize() {
		setHeight.whileTrue(Commands.runOnce(() -> elevator.setHeight(() -> targetHeight)));
		setAngle.whileTrue(Commands.run(() -> endeffector.setAngle(() -> targetAngle)));
		intakeRun.whileTrue(Commands.run(() -> endeffector.runIntake(volt_target))).whileFalse(Commands.run(() -> endeffector.runIntake(Volts.zero())));
		endeffector.stop();
	}

	@Override
	public void execute() {
		targetHeight = clamp(height.getDouble(0), 0, ElevatorConstants.Heights.Max);
		targetAngle = Degrees.of(clamp(angle.getDouble(0), EndEffectorConstants.minAngle.in(Degrees), EndEffectorConstants.maxAngle.in(Degrees)));
		volt_target = Volts.of(volts.getDouble(0));
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
