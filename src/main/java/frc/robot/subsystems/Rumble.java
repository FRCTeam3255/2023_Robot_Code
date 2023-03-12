// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences.prefControllers;

public class Rumble extends SubsystemBase {
  SN_XboxController conDriver;
  SN_XboxController conOperator;

  public Rumble(SN_XboxController conDriver, SN_XboxController conOperator) {
    this.conDriver = conDriver;
    this.conOperator = conOperator;
  }

  public void setInstantRumble() {
    conDriver.setRumble(RumbleType.kBothRumble, prefControllers.rumbleOutput.getValue());
    conOperator.setRumble(RumbleType.kBothRumble, prefControllers.rumbleOutput.getValue());
    Timer.delay(prefControllers.rumbleDelay.getValue());
    conDriver.setRumble(RumbleType.kBothRumble, 0);
    conDriver.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {

  }
}
