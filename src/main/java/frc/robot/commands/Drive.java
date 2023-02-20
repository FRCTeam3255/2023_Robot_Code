// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  DoubleSupplier xAxis;
  DoubleSupplier yAxis;
  DoubleSupplier rotationAxis;
  DoubleSupplier slowAxis;

  Translation2d translationVelocity;
  double translationScalar;

  Pose2d velocity;

  public Drive(
      Drivetrain subDrivetrain,
      DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis,
      DoubleSupplier slowAxis) {

    this.subDrivetrain = subDrivetrain;

    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowAxis = slowAxis;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    xVelocity = xAxis.getAsDouble() * Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    yVelocity = yAxis.getAsDouble() * Units.feetToMeters(prefDrivetrain.driveSpeed.getValue());
    rVelocity = rotationAxis.getAsDouble() * Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());
    translationScalar = SN_Math.interpolate(slowAxis.getAsDouble(), 0, 1, 1, prefDrivetrain.triggerValue.getValue());

    translationVelocity = new Translation2d(xVelocity, yVelocity).times(translationScalar);

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
