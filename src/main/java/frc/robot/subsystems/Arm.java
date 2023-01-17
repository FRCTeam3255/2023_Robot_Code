// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    shoulderEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);
    elbowEncoder.setDistancePerRotation(mapArm.DISTANCE_PER_ROTATION);

    m_pidController = shoulderJoint.getPIDController();

    // PID coefficients
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    shoulderJoint.getEncoder().setPositionConversionFactor(mapArm.ENCODER_CONVERSION_FACTOR);
    shoulderJoint.getEncoder().setPosition(shoulderEncoder.getDistance());

    elbowJoint.getEncoder().setPositionConversionFactor(mapArm.ENCODER_CONVERSION_FACTOR);
    elbowJoint.getEncoder().setPosition(elbowEncoder.getDistance());

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

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder abs - shoulder", shoulderEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder dist - shoulder", shoulderEncoder.getDistance());
    SmartDashboard.putNumber("motor encoder - shoulder", shoulderJoint.getEncoder().getPosition());

    SmartDashboard.putNumber("encoder abs - elbow", elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder dist - elbow", elbowEncoder.getDistance());
    SmartDashboard.putNumber("motor encoder - elbow", elbowJoint.getEncoder().getPosition());
  }
}
