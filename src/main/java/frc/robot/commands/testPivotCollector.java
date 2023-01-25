// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_DualActionStick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constControllers;
import frc.robot.subsystems.Collector;

public class testPivotCollector extends CommandBase {

  Collector subCollector;
  SN_DualActionStick conOperator;
  double velocity;

  public testPivotCollector(Collector subCollector, SN_DualActionStick conOperator) {
    this.subCollector = subCollector;
    this.conOperator = conOperator;

    addRequirements(subCollector);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    velocity = MathUtil.applyDeadband(conOperator.getRightStickX(), constControllers.OPERATOR_RIGHT_STICK_X_DEADBAND);
    subCollector.setPivotMotorSpeed(velocity);
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
