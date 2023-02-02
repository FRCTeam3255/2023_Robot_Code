// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constArm;
import frc.robot.Constants.constControllers;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  Arm subArm;
  SN_F310Gamepad conOperator;

  double x;
  double y;

  double controllerX;
  double controllerY;

  double lastX;
  double lastY;

  Translation2d origin = new Translation2d();

  double reach = constArm.SHOULDER_LENGTH + constArm.ELBOW_LENGTH;

  public MoveArm(Arm subArm, SN_F310Gamepad conOperator) {
    this.subArm = subArm;
    this.conOperator = conOperator;

    addRequirements(subArm);
  }

  @Override
  public void initialize() {
    x = subArm.getArmTipPosition().getX();
    y = subArm.getArmTipPosition().getY();
  }

  @Override
  public void execute() {

    lastX = x;
    lastY = y;

    controllerX = MathUtil.applyDeadband(conOperator.getAxisLSX(), constControllers.OPERATOR_LEFT_STICK_X_DEADBAND);
    controllerY = MathUtil.applyDeadband(conOperator.getAxisLSY(), constControllers.OPERATOR_LEFT_STICK_Y_DEADBAND);

    controllerX /= 50; // this code runs 50 times per second, so this makes it 1 time per second
    controllerY /= 50; // this code runs 50 times per second, so this makes it 1 time per second

    controllerX *= Units.inchesToMeters(prefArm.armTipSpeed.getValue());
    controllerY *= Units.inchesToMeters(prefArm.armTipSpeed.getValue());

    x += controllerX;
    y += controllerY;

    // if legit:
    // x,y += controller * rate pref
    // set position

    if (new Translation2d(x, y).getDistance(origin) < reach
        && new Translation2d(x, y).getDistance(origin) > Units.inchesToMeters(prefArm.armTipDeadzone.getValue())) {
      subArm.setArmTipPosition(new Translation2d(x, y));
    } else {
      subArm.setArmTipPosition(new Translation2d(lastX, lastY));
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
