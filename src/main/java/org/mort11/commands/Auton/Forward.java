package org.mort11.commands.Auton;

import org.mort11.commands.Called.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Forward extends SequentialCommandGroup {

	public Forward() {

		addCommands(new TimedDrive(10, 0, -1, 0));
	}
}
