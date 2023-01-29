// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class intakeCube extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  SN_Blinkin leds;

  public intakeCube(Collector subCollector, Intake subIntake, SN_Blinkin leds) {
    addCommands(
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.rollerHeightPivotAngle.getValue())),
        // Move arm to collector (waiting on #96 for this)
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeMotorSpeed)),
        // Spin intake
        new InstantCommand(() -> subCollector.spinRollerMotor(prefCollector.rollerSpeed.getValue())).until(null),
        // Spin rollers .until a game piece is collected (waiting on #125 for this)
        // detect the object (done periodically in the subsystem)
        // retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.startingConfigPivotAngle.getValue())),
        // LEDS! Note: Not sure if I should make a constant for this, because I don't
        // like setting it in here, but also, they can't be a preference so idk
        new InstantCommand(() -> leds.setPattern(PatternType.Violet)));
  }
}
