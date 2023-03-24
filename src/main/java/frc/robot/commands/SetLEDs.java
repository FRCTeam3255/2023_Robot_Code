// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefLEDs;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {
  LEDs subLEDs;
  Intake subIntake;
  Drivetrain subDrivetrain;
  Arm subArm;

  PatternType desiredPattern;

  Double chargeStationCenterX;
  Double chargeStationCenterToleranceX;
  Double chargeStationCenterY;
  Double chargeStationCenterToleranceY;

  int desiredColumn;

  public SetLEDs(LEDs subLEDs, Intake subIntake, Drivetrain subDrivetrain, Arm subArm) {
    this.subLEDs = subLEDs;
    this.subIntake = subIntake;
    this.subDrivetrain = subDrivetrain;
    this.subArm = subArm;

    chargeStationCenterX = prefVision.chargeStationCenterX.getValue();
    chargeStationCenterToleranceX = prefVision.chargeStationCenterToleranceX.getValue();
    chargeStationCenterY = prefVision.chargeStationCenterY.getValue();
    chargeStationCenterToleranceY = prefVision.chargeStationCenterToleranceY.getValue();

    addRequirements(subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Double[] coordinates = {};
    desiredColumn = subArm.getDesiredColumn();

    if (subIntake.isGamePieceCollected()) {
      desiredPattern = constLEDs.HAS_GAME_PIECE_COLOR;
    } else {
      desiredPattern = constLEDs.DEFAULT_COLOR;
    }

    if (Timer.getMatchTime() < prefLEDs.timeChargeStationLEDsOn.getValue()) {
      if (Math.abs(subDrivetrain.getPose().getX() - chargeStationCenterX) < chargeStationCenterToleranceX
          && Math.abs(subDrivetrain.getPose().getY() - chargeStationCenterY) < chargeStationCenterToleranceY) {
        desiredPattern = constLEDs.CHARGE_STATION_ALIGNED_COLOR;
      }
    }

    if (desiredColumn > 0) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        coordinates = subDrivetrain.columnYCoordinatesBlue;
      } else if (DriverStation.getAlliance() == Alliance.Red) {
        coordinates = subDrivetrain.columnYCoordinatesRed;
      }
      if (Math.abs(subDrivetrain.getPose().getY()
          - coordinates[desiredColumn - 1]) < prefVision.gridAlignmentToleranceY.getValue()
          && subDrivetrain.getPose().getX() < prefVision.gridLEDsXPosMax.getValue()) {
        desiredPattern = constLEDs.GRID_ALIGNED_COLOR;
      }
    }

    subLEDs.setLEDPattern(desiredPattern);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
