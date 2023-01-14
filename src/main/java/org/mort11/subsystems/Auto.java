package org.mort11.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;

public class Auto extends SubsystemBase {
    private Drivetrain drivetrain;
    private HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;

    private static Auto auto;

    public Auto() {
        drivetrain = Drivetrain.getInstance();
        createEventMap();

        autoBuilder =
                new SwerveAutoBuilder(
                        drivetrain::getPose, // Pose2d supplier
                        drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the
                        // beginning of auto
                        drivetrain.driveKinematics, // SwerveDriveKinematics
                        new PIDConstants(
                                0, 0.0,
                                0.0), // PID constants to correct for translation error (used to
                        // create the X and Y PID controllers)
                        new PIDConstants(
                                -0.5, 0.0,
                                0.0), // PID constants to correct for rotation error (used to create
                        // the rotation controller)
                        drivetrain::setModuleStates, // Module states consumer used to output to the
                        // drive subsystem
                        eventMap,
                        drivetrain);
    }

    private void createEventMap() {
        eventMap = new HashMap<String, Command>();
        eventMap.put("Example Command", new InstantCommand());
    }

    public Command createAutoCommand(ArrayList<PathPlannerTrajectory> pathGroup) {
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command createAutoCommand2(PathPlannerTrajectory traj) {
        drivetrain.resetPose(traj.getInitialHolonomicPose());
        return new PPSwerveControllerCommand(
                traj,
                drivetrain::getPose, // Pose supplier
                drivetrain.driveKinematics, // SwerveDriveKinematics
                new PIDController(
                        0.12, 0,
                        0), // X controller. Tune these values for your robot. Leaving them 0 will
                // only use feedforwards.
                new PIDController(
                        0.1, 0, 0.0), // Y controller (usually the same values as X controller)
                new PIDController(
                        0.1, 0,
                        0), // Rotation controller. Tune these values for your robot. Leaving them 0
                // will only use feedforwards.
                drivetrain::setModuleStates, // Module states consumer
                drivetrain // Requires this drive subsystem
                );
    }

    public static Auto getInstance() {
        if (auto == null) {
            auto = new Auto();
        }

        return auto;
    }
}
