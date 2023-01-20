// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot.
 * 
 * Left stick controls the direction and speed of translation.
 * Right trigger will slow the robot down as an analog input.
 * 
 * Right stick controls the rate of rotation.
 *
 */
public class Drive extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;

  // x and y velocity of robot in field relative coordinates
  double xVelocity;
  double yVelocity;

  // rotational velocity of robot
  double rVelocity;

  // translational velocity of robot (field relative)
  Translation2d translationVelocity;
  // value to multiply with translational velocity
  double translationScalar;

  // translational and rotational velocity of robot
  Pose2d velocity;

  public Drive(Drivetrain subDrivetrain, SN_F310Gamepad conDriver) {

    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // assign the velocity values to joysticks. due to the differing coordinate
    // systems between the robot and controller these don't line up
    xVelocity = -conDriver.getAxisLSY();
    yVelocity = conDriver.getAxisLSX();
    rVelocity = conDriver.getAxisRSX();

    // value to multiply translationalVelocity with
    translationScalar = conDriver.getAxisRT();

    // apply deadband to joystick inputs
    xVelocity = MathUtil.applyDeadband(xVelocity, constControllers.DRIVER_LEFT_STICK_Y_DEADBAND);
    yVelocity = MathUtil.applyDeadband(yVelocity, constControllers.DRIVER_LEFT_STICK_X_DEADBAND);
    rVelocity = MathUtil.applyDeadband(rVelocity, constControllers.DRIVER_RIGHT_STICK_X_DEADBAND);
    translationScalar = MathUtil.applyDeadband(translationScalar, constControllers.DRIVER_RIGHT_TRIGGER_DEADBAND);

    // scale joystick inputs to be correct units
    xVelocity *= Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    yVelocity *= Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    rVelocity *= Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());
    translationScalar = SN_Math.interpolate(translationScalar, 0, 1, 1, prefDrivetrain.triggerValue.getValue());

    translationVelocity = new Translation2d(xVelocity, yVelocity);
    velocity = new Pose2d(
        translationVelocity,
        new Rotation2d(rVelocity));

    subDrivetrain.drive(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
