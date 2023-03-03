package org.mort11.commands.Auton;

import org.mort11.commands.Called.AngleRotate;
import org.mort11.commands.Called.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Tester extends SequentialCommandGroup {

    public Tester() {

        addCommands(
            new SequentialCommandGroup(
                new AngleRotate(90)
            )
        );
    }
}
