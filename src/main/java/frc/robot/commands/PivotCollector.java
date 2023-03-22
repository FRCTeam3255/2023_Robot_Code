// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;

public class PivotCollector extends CommandBase {

  Collector subCollector;
  Arm subArm;

  SN_DoublePreference desiredPivotAngleDegrees;
  Rotation2d pivotAngle;

  public PivotCollector(Collector subCollector, Arm subArm, SN_DoublePreference desiredPivotAngleDegrees) {
    this.subCollector = subCollector;
    this.subArm = subArm;

    this.desiredPivotAngleDegrees = desiredPivotAngleDegrees;

    addRequirements(this.subCollector);
  }

  @Override
  public void initialize() {
    pivotAngle = subCollector.getPivotAngle();

    if (subArm.isCurrentState(ArmState.COLLECTOR_MOVING)) {
      pivotAngle = Rotation2d.fromDegrees(desiredPivotAngleDegrees.getValue());
    }

    subCollector.setPivotAngle(pivotAngle);
  }
}
