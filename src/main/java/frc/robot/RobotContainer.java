// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  // TODO: Update PWMChannel

  private final SN_Blinkin leds = new SN_Blinkin(0);

  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(2);

  public RobotContainer() {
    configureBindings();
  }

  // Leds

  // While held, Leds will change to given color, and turn off on release
  private void configureBindings() {

    // Switchboard
    conSwitchboard.btn_1
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Violet)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    conSwitchboard.btn_2
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Yellow)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
