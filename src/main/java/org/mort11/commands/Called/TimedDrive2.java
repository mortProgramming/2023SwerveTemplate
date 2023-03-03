package org.mort11.commands.Called;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedDrive2 extends CommandBase{
    private Drivetrain drivetrain;

    private Timer timer;
    private double time;

    private double xSpeed;
    private double ySpeed;
    private double rSpeed;

    public TimedDrive2(double time, double xSpeed, double ySpeed, double rSpeed) {
        drivetrain = Drivetrain.getInstance();

        timer = new Timer();
        this.time = time;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rSpeed = rSpeed;

        addRequirements(drivetrain);
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed,
				ySpeed,
				rSpeed,
				drivetrain.getGyroscopeRotation()));
    }

    public boolean isFinished() {
        return timer.get() > time;
    }

    public void end(boolean interupted){
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
