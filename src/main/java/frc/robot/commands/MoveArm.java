// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

public class MoveArm extends CommandBase {

  Arm subArm;
  Collector subCollector;

  public MoveArm(Arm subArm, Collector subCollector) {
    this.subArm = subArm;
    this.subCollector = subCollector;

    addRequirements(subArm);
  }

  @Override
  public void initialize() {
    subArm.setGoalArmTipPosition(subArm.getArmTipPosition());
  }

  @Override
  public void execute() {

    subArm.setArmTipPosition(subArm.getGoalArmTipPosition());

  }

  @Override
  public void end(boolean interrupted) {
    subArm.neutralJointOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
