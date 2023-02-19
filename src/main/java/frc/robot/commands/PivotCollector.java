// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.subsystems.Collector;

public class PivotCollector extends CommandBase {

  Collector subCollector;

  public PivotCollector(Collector subCollector) {
    this.subCollector = subCollector;

    addRequirements(this.subCollector);
  }

  @Override
  public void initialize() {
    subCollector.setGoalPosition(Rotation2d.fromRadians(subCollector.getPivotMotorPosition()));
  }

  @Override
  public void execute() {
    subCollector.setPivotMotorAngle(subCollector.getGoalPosition().getDegrees());

    if (subCollector.isPivotMotorInToleranceForRoller()
        && subCollector.getGoalPosition().getDegrees() == prefCollector.pivotAngleCubeCollecting.getValue()) {
      subCollector.setRollerMotorSpeed(prefCollector.rollerSpeed.getValue());
    } else {
      subCollector.setRollerMotorSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    subCollector.setPivotMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
