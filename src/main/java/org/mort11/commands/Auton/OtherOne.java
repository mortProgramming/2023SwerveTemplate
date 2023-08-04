package org.mort11.commands.Auton;

import org.mort11.commands.Called.TimedDrive2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OtherOne extends SequentialCommandGroup {

	public OtherOne() {

		addCommands(new SequentialCommandGroup(new TimedDrive2(4, 0, 1, 1)));
	}
}
