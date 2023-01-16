// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapArm;

public class Arm extends SubsystemBase {

  SN_CANSparkMax shoulderJoint;
  DutyCycleEncoder shoulderEncoder;
  SN_CANSparkMax elbowJoint;
  DutyCycleEncoder elbowEncoder;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_CAN);
    shoulderEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_CAN);
    elbowEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    shoulderJoint.getEncoder().setPositionConversionFactor(mapArm.ENCODER_CONVERSION_FACTOR);
    shoulderJoint.getEncoder().setPosition(shoulderEncoder.getDistance());

    elbowJoint.getEncoder().setPositionConversionFactor(mapArm.ENCODER_CONVERSION_FACTOR);
    elbowJoint.getEncoder().setPosition(elbowEncoder.getDistance());

    configure();
  }

  public void setShoulderPosition() {
    shoulderJoint.set(ControlMode.Position, mapArm.SHOULDER_POSITION_DEGREES);
  }

  public void setElbowPosition() {
    elbowJoint.set(ControlMode.Position, mapArm.ELBOW_POSITION_DEGREES);
  }

  public void resetEncoders() {
    shoulderJoint.getEncoder().setPosition(shoulderEncoder.getDistance());
  }

  public void configure() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder abs", shoulderEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder dist", shoulderEncoder.getDistance());
    SmartDashboard.putNumber("motor encoder", shoulderJoint.getEncoder().getPosition());

    SmartDashboard.putNumber("encoder abs", elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder dist", elbowEncoder.getDistance());
    SmartDashboard.putNumber("motor encoder", elbowJoint.getEncoder().getPosition());
  }
}
