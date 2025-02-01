// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.logging.Logger;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	// private final RobotContainer m_robotContainer;

	private AddressableLED led = new AddressableLED(0);
	private AddressableLEDBuffer buf = new AddressableLEDBuffer(16);
	private LEDPattern red = LEDPattern.solid(Color.kRed);

	public Robot() {
		// m_robotContainer = new RobotContainer();
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());

		led.setLength(buf.getLength());

		led.setData(buf);
		led.start();

		red.applyTo(buf);

		led.setData(buf);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		CompletableFuture.runAsync(() -> {
			Logger.getInstance().log(0);
		});
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		// m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		m_robotContainer.stopDefaultCommands();
		m_robotContainer.getTuningCommand().schedule();;
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
		m_robotContainer.setDefaultCommands();
	}
}
