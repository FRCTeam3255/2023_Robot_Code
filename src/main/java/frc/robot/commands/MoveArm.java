// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constControllers;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  Arm subArm;
  SN_F310Gamepad conOperator;

  double x;
  double y;

  boolean hasInput;

  public MoveArm(Arm subArm, SN_F310Gamepad conOperator) {
    this.subArm = subArm;
    this.conOperator = conOperator;

    x = 8;
    y = 8;

    addRequirements(subArm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    x = x + Math.pow(MathUtil.applyDeadband(conOperator.getAxisLSX(), constControllers.OPERATOR_LEFT_STICK_X_DEADBAND),
        3);
    y = y + Math.pow(MathUtil.applyDeadband(conOperator.getAxisLSY(), constControllers.OPERATOR_LEFT_STICK_Y_DEADBAND),
        3);

    subArm.setArmTipPosition(new Translation2d(x, y));

    System.out.println("*******************x: " + x + " y: " + y);
  }

  @Override
  public void end(boolean interrupted) {
    subArm.setJointPercentOutputs(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
