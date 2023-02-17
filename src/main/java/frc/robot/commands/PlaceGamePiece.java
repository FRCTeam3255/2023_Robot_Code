// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class PlaceGamePiece extends SequentialCommandGroup {
  Arm subArm;
  Intake subIntake;
  SN_DoublePreference shoulderLoweringDegrees;

  public PlaceGamePiece(Arm subArm, Intake subIntake) {
    this.subArm = subArm;
    this.subIntake = subIntake;
    shoulderLoweringDegrees = new SN_DoublePreference("shoulderLoweringDegrees",
        prefArm.armPresetHighShoulderAngle.getValue() - prefArm.shoulderLoweringAngle.getValue());

    addCommands(
        // - Set the arm to chosen numpad level

        // until someone figures out numpad logic, i put the high level as a placeholder
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetHighShoulderAngle, prefArm.armPresetHighElbowAngle)),

        // should there be a delay?

        // - Move arm down
        new InstantCommand(() -> subArm.setGoalAngles(shoulderLoweringDegrees, prefArm.armPresetHighElbowAngle)),

        // - Reverse the intake
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))

    // - Drive away???
    );
  }
}
