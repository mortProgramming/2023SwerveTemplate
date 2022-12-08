package org.mort11;

import static org.mort11.Constants.DrivetrainSpecs.*;
import static org.mort11.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.mort11.commands.DriveControl;
import org.mort11.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final XboxController xboxController = new XboxController(CONTROLLER_PORT);
    private final Joystick joystick = new Joystick(JOYSTICK_PORT);

    public RobotContainer() {
        drivetrain.setDefaultCommand(
                new DriveControl(
                        () ->
                                -modifyAxis(joystick.getY(), joystick.getThrottle())
                                        * MAX_VELOCITY_METERS_PER_SECOND,
                        () ->
                                -modifyAxis(joystick.getX(), joystick.getThrottle())
                                        * MAX_VELOCITY_METERS_PER_SECOND,
                        () ->
                                -modifyAxis(joystick.getTwist(), joystick.getThrottle())
                                        * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Button(joystick::getTrigger).whenPressed(new InstantCommand(drivetrain::zeroGyroscope));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new InstantCommand();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value, double throttleValue) {
        // Deadband
        value = deadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value * value, value);

        // takes the throttle value and takes it from [-1, 1] to [0.2, 1], and multiplies it by the
        // value
        return value * (throttleValue * -0.4 + 0.6);
        // return value * ((((-throttleValue + 1) / 2) + 0.2) / 1.2);
        // value * -throttleValue / 2.5 + 1.2; //throttle value from (-1,1) to (0.2,1)
    }
}