// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

public class MoveArm extends CommandBase {

  Arm subArm;
  Collector subCollector;

  SN_F310Gamepad conOperator;

  Rotation2d goalShoulderAngle;
  Rotation2d goalElbowAngle;

  public MoveArm(Arm subArm, Collector subCollector, SN_F310Gamepad conOperator) {
    this.subArm = subArm;
    this.subCollector = subCollector;

    this.conOperator = conOperator;

    addRequirements(subArm);
  }

  @Override
  public void initialize() {
    subArm.setGoalAngles(subArm.getShoulderPosition(), subArm.getElbowPosition());
  }

  @Override
  public void execute() {

    goalShoulderAngle = Rotation2d.fromDegrees(
        subArm.getGoalShoulderAngle().getDegrees()
            + (conOperator.getAxisLSX() * prefArm.shoulderAdjustRange.getValue()));

    goalElbowAngle = Rotation2d.fromDegrees(
        subArm.getGoalElbowAngle().getDegrees()
            + (conOperator.getAxisRSY() * prefArm.elbowAdjustRange.getValue()));

    subArm.setJointPositions(goalShoulderAngle, goalElbowAngle);

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
