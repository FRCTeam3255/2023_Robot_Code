// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.frcteam3255.joystick.SN_DualActionStick;
import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  // TODO: Update PWMChannel
  
  private final SN_Blinkin leds = new SN_Blinkin(0);

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(0);
  private final SN_DualActionStick conOperator = new SN_DualActionStick(1);

  public RobotContainer() {
    configureBindings();
  }

  // Leds

  // While held, Leds will change to given color, and turn off on release
  private void configureBindings() {

    // Driver
    conDriver.btn_X
      .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Yellow)))
      .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    conDriver.btn_Y
      .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Violet)))
      .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    // Operator
    conOperator.btn_A
      .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Yellow)))
      .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    conOperator.btn_B
      .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Violet)))
      .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
