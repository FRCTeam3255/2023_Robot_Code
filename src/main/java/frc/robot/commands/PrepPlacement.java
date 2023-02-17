// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class PrepPlacement extends SequentialCommandGroup {

  // TODO: Make sure only using Y axis
  Arm arm;
  ColorSensorV3 colorSensor;
  Drivetrain drivetrain;
  Intake intake;

  public PrepPlacement(Arm subArm, ColorSensorV3 colorSensor, Drivetrain subDrivetrain, Intake subIntake) {
    addCommands(
        // Get game piece type via color sensor
        new InstantCommand(() -> subIntake.getGamePieceType())

    // Get operator input via num pad

    // If position is valid, drive and move the arm on the Y-axis

    // TODO: Make sure to assign button binding for driver

    );
  }
}
