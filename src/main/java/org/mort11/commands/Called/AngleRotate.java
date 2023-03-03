package org.mort11.commands.Called;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AngleRotate extends CommandBase{
    private Drivetrain drivetrain;

    private static double angle;

    private static double initialYaw;

    public AngleRotate(double angle) {
        drivetrain = Drivetrain.getInstance();

        this.angle = angle;

        addRequirements(drivetrain);
    }

    public void initial() {
        initialYaw = drivetrain.getAngle();
    }

    public void execute() {
		drivetrain.drive(new ChassisSpeeds(0, 0, 
				drivetrain.angleRotateController().calculate(-drivetrain.getAngle(), angle + initialYaw)
		));
    }

    @Override
    public boolean isFinished() {
        if(drivetrain.angleRotateSetpoint()){
            System.out.println("finished");
            return true;
        }
        else{
            return false;
        }
    }

    public void end(boolean interupted){
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
