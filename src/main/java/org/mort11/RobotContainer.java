package org.mort11;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.mort11.util.Constants.DrivetrainSpecs.*;
import org.mort11.util.Constants;

import java.util.ArrayList;
import java.util.HashMap;
import org.mort11.util.Control;
import org.mort11.commands.Control.DriveControl;
import org.mort11.commands.Sumeet.BalanceStation;
import org.mort11.commands.Sumeet.DriveToAprilTag;
import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.Limelight;
import org.mort11.util.Auto;

public class RobotContainer {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	// private final Auto auto = Auto.getInstance();
	private final Limelight limelight = Limelight.getInstance();

	// private final XboxController xboxController = new
	// XboxController(CONTROLLER_PORT);
	private final Joystick joystick = new Joystick(Constants.LEFT_JOYSTICK);

	private DigitalInput sensor;

	public RobotContainer() {
		Control.init();

		drivetrain.setDefaultCommand(new DriveControl(
				() -> -modifyAxis(-joystick.getX(), joystick.getThrottle()) * MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(joystick.getY(), joystick.getThrottle()) * MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(joystick.getTwist(), joystick.getThrottle())
						* MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

		Control.configureBindings();
		drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0, 0)));

		sensor = new DigitalInput(0);
		// drivetrain.zeroGyroscope();
		Auto.init();
	}

	public void displaySmartDashboard() {
		SmartDashboard.putNumber("compass", drivetrain.getCompass());
		SmartDashboard.putNumber("angle", drivetrain.getGyroscopeRotation().getDegrees());
		SmartDashboard.putNumber("yaw", drivetrain.getNavX().getYaw());
		SmartDashboard.putNumber("pitch", drivetrain.getNavX().getPitch());
		SmartDashboard.putNumber("roll", drivetrain.getNavX().getRoll());

		SmartDashboard.putBoolean("ir sensor", sensor.get());

	}

	/**
	 *
	 */
	// private void configureButtonBindings() {
	// // new Button(joystick::getTrigger).whenPressed(new
	// // InstantCommand(drivetrain::zeroGyroscope));
	// new Trigger(joystick::getTrigger).whileTrue(new
	// InstantCommand(drivetrain::zeroGyroscope));
	// new Trigger(() -> joystick.getRawButton(2)).whileTrue(new BalanceStation());
	// new Trigger(() -> joystick.getRawButton(3)).whileTrue(
	// new InstantCommand(() -> drivetrain.drive(new
	// ChassisSpeeds(SmartDashboard.getNumber("vx", 0),
	// SmartDashboard.getNumber("vy", 0), SmartDashboard.getNumber("omega", 0)))));

	// // new Trigger(joystick::getTrigger).whileTrue(new DriveToAprilTag(1));
	// // new Trigger(joystick::getTrigger).whileTrue(new RotateToAngle(90, false));

	// }

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

		// takes the throttle value and takes it from [-1, 1] to [0.2, 1], and
		// multiplies it by the
		// value
		return value * (throttleValue * -0.4 + 0.6);
	}

	private static double modifyAxis2(double value, double throttleValue) {
		// Deadband
		value = deadband(value, 0.1);

		// Square the axis
		value = Math.copySign(value * value, value);

		throttleValue = (throttleValue + 1) / 2;

		double minValue = 0.2;
		double maxValue = 0.6;
		return value * (throttleValue * (maxValue - minValue) + minValue);
	}

	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return Auto.getSelected();
	}
}
