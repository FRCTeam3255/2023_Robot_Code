// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

public class RunRoller extends CommandBase {

  Collector subCollector;
  Arm subArm;

  public RunRoller(Collector subCollector, Arm subArm) {
    this.subCollector = subCollector;
    this.subArm = subArm;

    addRequirements(subCollector);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subArm.isCurrentState(ArmState.COLLECTOR_COLLECTING)
        && subCollector.getPivotAngle().getDegrees() > prefCollector.rollerThreshold.getValue()) {
      subCollector.setRollerSpeed(prefCollector.rollerSpeed);
    } else {
      subCollector.setRollerSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
