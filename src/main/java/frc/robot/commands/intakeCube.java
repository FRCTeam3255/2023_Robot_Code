// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class intakeCube extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  Arm subArm;
  SN_Blinkin leds;

  public intakeCube(Arm subArm, Collector subCollector, Intake subIntake, SN_Blinkin leds) {
    addCommands(
        // - Deploy collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleCubeCollecting.getValue())),

        // - Move arm to collector (waiting on #96 for this)
        new InstantCommand(
            () -> subArm.setArmTipPositionInches(prefArm.armTipToCollectorX, prefArm.armTipToCollectorY)),

        // Spin intake & rollers until a game piece is collected
        Commands.parallel(
            new IntakeGamePiece(subIntake),
            new InstantCommand(() -> subCollector.setRollerMotorSpeed(prefCollector.rollerSpeed.getValue()))
                .until(subIntake::isGamePieceCollected)),

        // Stop all motors
        Commands.parallel(
            new InstantCommand(() -> subCollector.setRollerMotorSpeed(0)),
            new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed))),

        // - raise arm to mid shelf position -- Maybe do something here to check if we
        // actually have a cube before continuing?
        new InstantCommand(
            () -> subArm.setArmTipPositionInches(prefArm.armTipToMidPosX, prefArm.armTipToMidPosY)),

        // - retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // - LEDS! Note: Not sure if I should make a constant for this, because I don't
        // like setting it in here, but also, they can't be a preference so idk
        new InstantCommand(() -> leds.setPattern(PatternType.Violet)));
  }
}
