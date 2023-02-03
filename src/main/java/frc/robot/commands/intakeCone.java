// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class intakeCone extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  SN_Blinkin leds;

  public intakeCone() {
    addCommands(

        // - Retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // Spin intake until a game piece is collected
        new InstantCommand(() -> subIntake.setMotorSpeed(0)).until(subIntake::isGamePieceCollected),

        // Stop motors
        new InstantCommand(() -> subIntake.setMotorSpeed(0)),

        // - raise arm to mid node position

        // - LEDS: May want to be a constant since they can't be set as a preference
        new InstantCommand(() -> leds.setPattern(PatternType.Yellow)));
  }
}