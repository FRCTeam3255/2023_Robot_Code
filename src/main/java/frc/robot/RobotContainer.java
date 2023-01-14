// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.mapControllers;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
