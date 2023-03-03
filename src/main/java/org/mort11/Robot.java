package org.mort11;

import org.mort11.util.Auto;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		PathPlannerServer.startServer(5811);

		SmartDashboard.putNumber("vx", 0);
		SmartDashboard.putNumber("vy", 0);
		SmartDashboard.putNumber("omega", 0);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		m_robotContainer.displaySmartDashboard();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	public void autonomousInit() {
		// m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		m_autonomousCommand = Auto.getSelected();
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
		  m_autonomousCommand.schedule();
		}
	  }

	@Override
	public void autonomousPeriodic() {
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
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}
}
