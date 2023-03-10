// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  Arm subArm;

  DoubleSupplier shoulderAdjuster;
  DoubleSupplier elbowAdjuster;

  Rotation2d shoulderAngle;
  Rotation2d elbowAngle;

  public MoveArm(Arm subArm, DoubleSupplier shoulderAdjuster, DoubleSupplier elbowAdjuster) {
    this.subArm = subArm;

    this.shoulderAdjuster = shoulderAdjuster;
    this.elbowAdjuster = elbowAdjuster;

    addRequirements(subArm);
  }

  @Override
  public void initialize() {
    subArm.setGoalAnglesToCurrentAngles();
  }

  @Override
  public void execute() {

    switch () {
      case ArmState:

        break;

      default:
        break;
    }

    shoulderAngle = Rotation2d.fromDegrees(
        subArm.getGoalShoulderAngle().getDegrees()
            + (shoulderAdjuster.getAsDouble() * prefArm.shoulderAdjustRange.getValue()));

    elbowAngle = Rotation2d.fromDegrees(
        subArm.getGoalElbowAngle().getDegrees()
            + (elbowAdjuster.getAsDouble() * prefArm.elbowAdjustRange.getValue()));

    subArm.setJointPositions(shoulderAngle, elbowAngle);

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
