// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class intakeCone extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  Arm subArm;
  SN_Blinkin leds;

  public intakeCone() {
    addCommands(

        // - Retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // - Lower the arm so that the intake is cone level
        new InstantCommand(
            () -> subArm.setArmTipPositionInches(prefArm.armTipToConeLevelX, prefArm.armTipToConeLevelY)),

        // - Spin intake until a game piece is collected
        new IntakeGamePiece(subIntake).until(subIntake::isGamePieceCollected),

        // - Set motors to hold speed
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed)),

        // - Raise arm to mid node position
        new InstantCommand(
            () -> subArm.setArmTipPositionInches(prefArm.armTipToMidNodePosX, prefArm.armTipToMidNodePosY)),

        // - Set LEDs to Yellow
        new InstantCommand(() -> leds.setPattern(Constants.INTAKE_CONE_LED_PATTERN)));
  }
}