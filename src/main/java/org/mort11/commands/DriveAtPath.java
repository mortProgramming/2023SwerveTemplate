package org.mort11.commands;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAtPath extends CommandBase {
    private Drivetrain drivetrain;
    private Trajectory trajectory;
    private HolonomicDriveController holonomicController;
    private Timer timer;
    private Rotation2d endRotation;

    public DriveAtPath(Trajectory traj, Rotation2d rotation) {
        drivetrain = Drivetrain.getInstance();
        
        trajectory = traj;
        
        holonomicController = new HolonomicDriveController(
                    new PIDController(0, 0, 0),
                    new PIDController(0, 0, 0),
                    new ProfiledPIDController(1, 0, 0,
                        new TrapezoidProfile.Constraints(2, 1)
                    )
        );

        timer = new Timer();

        endRotation = rotation;        
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.drive(holonomicController.calculate(drivetrain.getPose(), trajectory.sample(timer.get()), endRotation));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getPose().getTranslation().getDistance(trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation()) < 0.01;
    }
}
