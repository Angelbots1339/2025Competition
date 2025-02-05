// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.configs.SlotConfigs;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeTuning extends Command {
	private Intake intake;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Intake Tuning");
	private static ShuffleboardLayout pid = tab.getLayout("PID", BuiltInLayouts.kList);
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static Voltage volt_target = Volt.zero();
	private static Angle targetAngle = Degrees.zero();

	private static GenericEntry target = tab.add("target", IntakeConstants.outsideAngle.in(Degrees))
			.withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 90))
			.getEntry();

	private static GenericEntry volts = tab.add("volts", 1)
			.withWidget(BuiltInWidgets.kTextView)
			.getEntry();

	private static GenericEntry p = pid.add("P", IntakeConstants.pid.kP)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
	private static GenericEntry i = pid.add("I", IntakeConstants.pid.kI)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
	private static GenericEntry d = pid.add("D", IntakeConstants.pid.kD)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
	private static GenericEntry g = pid.add("G", IntakeConstants.pid.kG)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
	private static GenericEntry s = pid.add("S", IntakeConstants.pid.kS)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();

	private static Trigger intakeAngle = new Trigger(() -> test.getBButton());
	private static Trigger angleUp = new Trigger(() -> test.getPOV() == 0);
	private static Trigger angleDown = new Trigger(() -> test.getPOV() == 180);
	private static Trigger intakeRun = new Trigger(() -> test.getAButton());

	public IntakeTuning(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void initialize() {
		intakeAngle.whileTrue(Commands.run(() -> intake.changeAngle(() -> targetAngle)));
		angleUp.onTrue(Commands.runOnce(() -> target.setDouble(Math.min(target.getDouble(0) + 5, IntakeConstants.insideAngle.in(Degrees)))));
		angleDown.onTrue(Commands.runOnce(() -> target.setDouble(Math.max(target.getDouble(0) - 5, IntakeConstants.outsideAngle.in(Degrees)))));

		intakeRun.whileTrue(Commands.run(() -> intake.runWheelsVolts(volt_target))).whileFalse(Commands.run(() -> intake.runWheelsVolts(Volt.zero())));
	}

	@Override
	public void execute() {
		targetAngle = Degrees.of(target.getDouble(0));

		volt_target = Volts.of(volts.getDouble(0));

		SlotConfigs tmp = IntakeConstants.pid
				.withKP(p.getDouble(0))
				.withKI(i.getDouble(0))
				.withKD(d.getDouble(0))
				.withKG(g.getDouble(0))
				.withKS(s.getDouble(0));

		intake.setPID(tmp);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
