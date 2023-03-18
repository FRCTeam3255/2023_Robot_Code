// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Engage extends CommandBase {

  Drivetrain subDrivetrain;

  boolean isDriveOpenLoop;

  public Engage(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    isDriveOpenLoop = false;

    addRequirements(subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subDrivetrain.isTiltedForward()) {
      subDrivetrain.drive(new Pose2d(Units.metersToFeet(prefDrivetrain.dockingSpeed.getValue()), 0, new Rotation2d()),
          isDriveOpenLoop);
    } else if (subDrivetrain.isTiltedBackwards()) {
      subDrivetrain.drive(new Pose2d(-Units.metersToFeet(prefDrivetrain.dockingSpeed.getValue()), 0, new Rotation2d()),
          isDriveOpenLoop);
    } else {
      subDrivetrain.drive(new Pose2d(0, 0, new Rotation2d()), isDriveOpenLoop);
      // defence mode here too would be nice
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
