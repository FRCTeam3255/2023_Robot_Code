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

  double xVelocity;
  double yVelocity;
  double rVelocity;

  Translation2d translationVelocity;
  double translationScalar;

  boolean isRotationPositional;
  Rotation2d positionalRotation;

  Pose2d velocity;
  Pose2d velocityRotation;

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

    isRotationPositional = false;

    if (conDriver.btn_A.getAsBoolean()) {
      isRotationPositional = true;
      positionalRotation = Rotation2d.fromDegrees(180);
    }

    if (conDriver.btn_B.getAsBoolean()) {
      isRotationPositional = true;
      positionalRotation = Rotation2d.fromDegrees(-90);
    }

    if (conDriver.btn_X.getAsBoolean()) {
      isRotationPositional = true;
      positionalRotation = Rotation2d.fromDegrees(90);
    }

    if (conDriver.btn_Y.getAsBoolean()) {
      isRotationPositional = true;
      positionalRotation = Rotation2d.fromDegrees(0);
    }

    xVelocity = conDriver.getAxisLSY();
    yVelocity = -conDriver.getAxisLSX();
    rVelocity = -conDriver.getAxisRSX();
    translationScalar = conDriver.getAxisRT();

    xVelocity = MathUtil.applyDeadband(xVelocity, constControllers.DRIVER_LEFT_STICK_Y_DEADBAND);
    yVelocity = MathUtil.applyDeadband(yVelocity, constControllers.DRIVER_LEFT_STICK_X_DEADBAND);
    rVelocity = MathUtil.applyDeadband(rVelocity, constControllers.DRIVER_RIGHT_STICK_X_DEADBAND);
    translationScalar = MathUtil.applyDeadband(translationScalar, constControllers.DRIVER_RIGHT_TRIGGER_DEADBAND);

    if (rVelocity > 0) {
      isRotationPositional = false;
    }

    xVelocity *= Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    yVelocity *= Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    rVelocity *= Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());
    translationScalar = SN_Math.interpolate(translationScalar, 0, 1, 1, prefDrivetrain.triggerValue.getValue());

    translationVelocity = new Translation2d(xVelocity, yVelocity).times(translationScalar);

    if (isRotationPositional) {
      velocityRotation = new Pose2d(
          translationVelocity,
          positionalRotation);

      subDrivetrain.driveAlignAngle(velocityRotation);
    } else {
      velocity = new Pose2d(
          translationVelocity,
          new Rotation2d(rVelocity));

      subDrivetrain.drive(velocity);
    }
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
