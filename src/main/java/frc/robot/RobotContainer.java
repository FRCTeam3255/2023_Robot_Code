// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Intake subIntake = new Intake();
  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD);
  private final SN_Blinkin leds = new SN_Blinkin(mapControllers.BLINKIN);

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(new Drive(subDrivetrain, conDriver));

    configureBindings();
  }

  // Leds

  // While held, Leds will change to given color, and turn off on release
  private void configureBindings() {

    // Switchboard

    // Sets LED color to "violet" to indicate a purple game piece (cube) is being
    // requested
    conSwitchboard.btn_1
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Violet)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    // Sets LED color to "yellow" to indicate a yellow game piece (cone) is being
    // requested
    conSwitchboard.btn_2
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Yellow)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
