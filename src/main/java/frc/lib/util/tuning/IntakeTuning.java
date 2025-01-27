// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class IntakeTuning extends Command {
	Intake intake;

	private static ShuffleboardTab tab = Shuffleboard.getTab("Intake Tuning");
	private static ShuffleboardLayout pid = tab.getLayout("PID", BuiltInLayouts.kList);
	private static XboxController test = new XboxController(DriverConstants.testPort);

	private static GenericEntry p, i, d, g, s;

	static {
		p = pid.add("P", IntakeConstants.pid.kP)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
		i = pid.add("I", IntakeConstants.pid.kI)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
		d = pid.add("D", IntakeConstants.pid.kD)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
		g = pid.add("G", IntakeConstants.pid.kG)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
		s = pid.add("S", IntakeConstants.pid.kS)
			.withWidget(BuiltInWidgets.kNumberSlider)
			.getEntry();
	}

	public IntakeTuning(Intake intake) {
		this.intake = intake;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		SlotConfigs tmp = IntakeConstants.pid
			.withKP(p.getDouble(0))
			.withKI(i.getDouble(0))
			.withKD(d.getDouble(0))
			.withKG(g.getDouble(0))
			.withKS(s.getDouble(0));

		intake.setPID(tmp);

		SmartDashboard.putNumber("p", intake.getPID());
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
