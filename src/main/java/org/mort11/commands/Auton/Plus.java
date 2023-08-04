package org.mort11.commands.Auton;

import org.mort11.commands.Called.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Plus extends SequentialCommandGroup {

	public Plus() {

		addCommands(new TimedDrive(1, 1, 0, 0), new TimedDrive(1, -1, 0, 0), new TimedDrive(1, 0, 1, 0),
				new TimedDrive(1, 0, -1, 0), new TimedDrive(1, -1, 0, 0), new TimedDrive(1, 1, 0, 0),
				new TimedDrive(1, 0, -1, 0), new TimedDrive(1, 0, 1, 0), new TimedDrive(1.7, 0, 0, 4));
	}
}
