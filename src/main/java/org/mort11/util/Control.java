package org.mort11.util;

import org.mort11.commands.Sumeet.BalanceStation;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Control {
	private static CommandJoystick leftJoystick;
	private static CommandJoystick rightJoystick;
	private static CommandXboxController xboxController;

	private static Drivetrain drivetrain;

	public static void init() {
		leftJoystick = new CommandJoystick(Constants.LEFT_JOYSTICK);
		rightJoystick = new CommandJoystick(Constants.RIGHT_JOYSTICK);
		xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER);

		drivetrain = Drivetrain.getInstance();
	}

	public static void configureBindings() {
		// new Button(joystick::getTrigger).whenPressed(new
		// InstantCommand(drivetrain::zeroGyroscope));
		// new Trigger(joystick::getTrigger).whileTrue(new
		// InstantCommand(drivetrain::zeroGyroscope));
		// new Trigger(() -> joystick.getRawButton(2)).whileTrue(new BalanceStation());
		// new Trigger(() -> joystick.getRawButton(3)).whileTrue(
		// new InstantCommand(() -> drivetrain.drive(new
		// ChassisSpeeds(SmartDashboard.getNumber("vx", 0),
		// SmartDashboard.getNumber("vy", 0), SmartDashboard.getNumber("omega", 0)))));

		// new Trigger(joystick::getTrigger).whileTrue(new DriveToAprilTag(1));
		// new Trigger(joystick::getTrigger).whileTrue(new RotateToAngle(90, false));

		leftJoystick.trigger().whileTrue(new InstantCommand(drivetrain::zeroGyroscope));
		leftJoystick.button(2).whileTrue(new BalanceStation());
	}

	public static double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	public static double getLeftThrottle() {
		return (0.5 * (1 - leftJoystick.getThrottle()));
	}
	public static double getRightJoystickY() {
		return rightJoystick.getY();
	}
	public static double getRightThrottle() {
		return (0.5 * (1 - rightJoystick.getThrottle()));
	}

	public static double leftControllerJoystickY() {
		return xboxController.getLeftY();
	}
}
