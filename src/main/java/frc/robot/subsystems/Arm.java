// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapArm;

public class Arm extends SubsystemBase {

  SN_CANSparkMax shoulderJoint;
  DutyCycleEncoder shoulderEncoder;
  SN_CANSparkMax elbowJoint;
  DutyCycleEncoder elbowEncoder;
  SparkMaxPIDController m_pidController;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    shoulderEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);
    elbowEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    m_pidController = shoulderJoint.getPIDController();

    // Change QuadEncoder to necessary input
    BaseTalonConfiguration config = new BaseTalonConfiguration(FeedbackDevice.QuadEncoder);

    shoulderJoint.configAllSettings(config);
    elbowJoint.configAllSettings(config);

    configure();
  }

  public void setShoulderPosition(double position) {
    shoulderJoint.set(ControlMode.Position, position);
  }

  public void setElbowPosition(double position) {
    elbowJoint.set(ControlMode.Position, position);
  }

  public void motorgobrr(double brrValue) {
    shoulderJoint.set(ControlMode.PercentOutput, brrValue);
  }

  public void resetEncoders() {
    shoulderJoint.getEncoder().setPosition(shoulderEncoder.getDistance());
  }

  public void configure() {
    shoulderJoint.configFactoryDefault();
    elbowJoint.configFactoryDefault();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shoulder encoder abs", shoulderEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("shoulder encoder dist", shoulderEncoder.getDistance());
    SmartDashboard.putNumber("shoulder motor encoder", shoulderJoint.getEncoder().getPosition());

    SmartDashboard.putNumber("elbow encoder abs", elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("elbow encoder dist", elbowEncoder.getDistance());
    SmartDashboard.putNumber("elbow motor encoder", elbowJoint.getEncoder().getPosition());
  }
}
