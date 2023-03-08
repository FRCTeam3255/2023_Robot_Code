// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Arm;
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
  Arm subArm;

  double xVelocity;
  double yVelocity;
  double rVelocity;

  DoubleSupplier xAxis;
  DoubleSupplier yAxis;
  DoubleSupplier rotationAxis;
  DoubleSupplier slowAxis;

  Trigger northTrigger;
  Trigger eastTrigger;
  Trigger southTrigger;
  Trigger westTrigger;

  Translation2d translationVelocity;
  double translationScalar;

  Pose2d velocity;

  boolean isRotationPositional;
  Rotation2d rotationPosition;
  Rotation2d lastRotationPosition;

  public Drive(
      Drivetrain subDrivetrain,
      Arm subArm,
      DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis,
      DoubleSupplier slowAxis,
      Trigger northTrigger,
      Trigger eastTrigger,
      Trigger southTrigger,
      Trigger westTrigger) {

    this.subDrivetrain = subDrivetrain;
    this.subArm = subArm;

    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowAxis = slowAxis;
    this.northTrigger = northTrigger;
    this.eastTrigger = eastTrigger;
    this.southTrigger = southTrigger;
    this.westTrigger = westTrigger;

    isRotationPositional = false;
    rotationPosition = new Rotation2d();
    lastRotationPosition = new Rotation2d();

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    isRotationPositional = false;
    rotationPosition = new Rotation2d();
  }

  @Override
  public void execute() {

    // get all the joystick axes and scale them to the correct units
    xVelocity = xAxis.getAsDouble() * Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    yVelocity = -yAxis.getAsDouble() * Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    rVelocity = -rotationAxis.getAsDouble() * Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());
    translationScalar = SN_Math.interpolate(slowAxis.getAsDouble(), 0, 1, 1, prefDrivetrain.triggerValue.getValue());

    // scale down the translation velocity from the driver input
    translationVelocity = new Translation2d(xVelocity, yVelocity).times(translationScalar);

    // if the driver is moving the rotation joystick just listen to that input,
    // don't do any positional rotation
    if (Math.abs(rVelocity) > 0) {
      isRotationPositional = false;
    }
    // if the driver isn't moving the rotation joystick, check if they pressed any
    // of the rotation buttons. each button corresponds to a cardinal direction
    else {

      if (northTrigger.getAsBoolean()) {
        isRotationPositional = true;
        rotationPosition = Rotation2d.fromDegrees(0);
      }

      if (eastTrigger.getAsBoolean()) {
        isRotationPositional = true;
        rotationPosition = Rotation2d.fromDegrees(-90);
      }

      if (southTrigger.getAsBoolean()) {
        isRotationPositional = true;
        rotationPosition = Rotation2d.fromDegrees(180);
      }

      if (westTrigger.getAsBoolean()) {
        isRotationPositional = true;
        rotationPosition = Rotation2d.fromDegrees(90);
      }

    }

    // if the driver isn't using the rotation joystick and also pressed a rotation
    // button, use driveAlignAngle for positional rotation control
    if (isRotationPositional) {
      velocity = new Pose2d(
          translationVelocity,
          rotationPosition);
      subDrivetrain.driveAlignAngle(velocity);
    }
    // if the driver didn't press any rotation buttons or used the rotation
    // joystick, just drive normally with the rotation joystick controlling the rate
    // of rotation
    else {
      velocity = new Pose2d(
          translationVelocity,
          Rotation2d.fromRadians(rVelocity));
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
