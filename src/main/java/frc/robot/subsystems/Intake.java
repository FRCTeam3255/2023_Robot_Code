// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {

  ColorSensorV3 intakeColorSensor;

  public Intake() {
    intakeColorSensor = new ColorSensorV3(mapIntake.COLOR_SENSOR_I2C);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color Sensor Color", intakeColorSensor.getColor().toString());
    SmartDashboard.putNumber("Color Sensor Red", intakeColorSensor.getRed());
    SmartDashboard.putNumber("Color Sensor Green", intakeColorSensor.getGreen());
    SmartDashboard.putNumber("Color Sensor Blue", intakeColorSensor.getBlue());
    SmartDashboard.putNumber("Color Sensor Proximity", intakeColorSensor.getProximity());
  }
}
