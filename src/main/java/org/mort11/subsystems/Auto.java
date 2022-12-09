package org.mort11.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auto extends SubsystemBase {
    private Drivetrain drivetrain;
    private HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;

    private static Auto auto;

    public Auto() {
        drivetrain = Drivetrain.getInstance();
        createEventMap();
        
        autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose, // Pose2d supplier
            drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            drivetrain.driveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            drivetrain
        );
    }
    
    private void createEventMap() {
        eventMap = new HashMap<String, Command>();
        eventMap.put("Example Command", new InstantCommand());
    }
    
    public Command createAutoCommand(ArrayList<PathPlannerTrajectory> pathGroup) {
        return autoBuilder.fullAuto(pathGroup);
    }

    public static Auto getInstance() {
        if (auto == null) {
            auto = new Auto();
        }

        return auto;
    }
    
}
