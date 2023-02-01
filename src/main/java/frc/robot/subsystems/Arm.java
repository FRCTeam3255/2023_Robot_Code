// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constArm;
import frc.robot.RobotMap.mapArm;
import frc.robot.RobotPreferences.prefArm;

public class Arm extends SubsystemBase {

  SN_CANSparkMax shoulderJoint;
  SN_CANSparkMax elbowJoint;

  TalonFXConfiguration shoulderConfig;
  TalonFXConfiguration elbowConfig;

  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder elbowEncoder;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);

    shoulderConfig = new TalonFXConfiguration();
    elbowConfig = new TalonFXConfiguration();

    configure();
  }

  public void configure() {

    // shoulder config
    shoulderJoint.configFactoryDefault();

    // this will set the encoder counts per rotation to be the same as the falcon,
    // so we can use the falcon conversion methods in SN_Math. (i think this must be
    // set before setting the forward and reverse limits)
    shoulderJoint.encoder
        .setPositionConversionFactor(SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    shoulderConfig.slot0.kP = prefArm.shoulderP.getValue();
    shoulderConfig.slot0.kI = prefArm.shoulderI.getValue();
    shoulderConfig.slot0.kD = prefArm.shoulderD.getValue();
    shoulderConfig.slot0.closedLoopPeakOutput = prefArm.shoulderMaxSpeed.getValue();

    shoulderConfig.forwardSoftLimitEnable = prefArm.shoulderForwardSoftLimitEnable.getValue();
    shoulderConfig.reverseSoftLimitEnable = prefArm.shoulderReverseSoftLimitEnable.getValue();

    shoulderConfig.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constArm.SHOULDER_FORWARD_LIMIT),
        constArm.SHOULDER_GEAR_RATIO);

    shoulderConfig.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constArm.SHOULDER_REVERSE_LIMIT),
        constArm.SHOULDER_GEAR_RATIO);

    shoulderJoint.configAllSettings(shoulderConfig);

    shoulderJoint.setInverted(constArm.SHOULDER_MOTOR_INVERT);
    shoulderJoint.setNeutralMode(constArm.SHOULDER_MOTOR_BREAK);

    // elbow config
    elbowJoint.configFactoryDefault();

    elbowJoint.encoder
        .setPositionConversionFactor(SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    elbowConfig.slot0.kP = prefArm.elbowP.getValue();
    elbowConfig.slot0.kI = prefArm.elbowI.getValue();
    elbowConfig.slot0.kD = prefArm.elbowD.getValue();
    elbowConfig.slot0.closedLoopPeakOutput = prefArm.elbowMaxSpeed.getValue();

    elbowConfig.forwardSoftLimitEnable = prefArm.elbowForwardSoftLimitEnable.getValue();
    elbowConfig.reverseSoftLimitEnable = prefArm.elbowReverseSoftLimitEnable.getValue();

    elbowConfig.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constArm.ELBOW_FORWARD_LIMIT),
        constArm.ELBOW_GEAR_RATIO);

    elbowConfig.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constArm.ELBOW_REVERSE_LIMIT),
        constArm.ELBOW_GEAR_RATIO);

    elbowConfig.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(
        prefArm.elbowTolerance.getValue(),
        constArm.ELBOW_GEAR_RATIO);

    elbowJoint.configAllSettings(elbowConfig);

    elbowJoint.setInverted(constArm.ELBOW_MOTOR_INVERT);
    elbowJoint.setNeutralMode(constArm.ELBOW_MOTOR_BREAK);

    // general config
    resetJointsToAbsolute();
  }

  public void setJointPositions(double shoulderAngle, double elbowAngle) {
    setShoulderPosition(shoulderAngle);
    setElbowPosition(elbowAngle - shoulderAngle);
  }

  public void setJointPositions(SN_DoublePreference shoulderAngle, SN_DoublePreference elbowAngle) {
    setJointPositions(shoulderAngle.getValue(), elbowAngle.getValue());
  }

  public void setShoulderPosition(double degrees) {
    if (Math.abs(getShoulderPosition().getDegrees() - degrees) > prefArm.shoulderTolerance.getValue()) {
      double encoderCounts = SN_Math.degreesToFalcon(
          degrees, constArm.SHOULDER_GEAR_RATIO);
      shoulderJoint.set(ControlMode.Position, encoderCounts);
    } else {
      shoulderJoint.neutralOutput();
    }
  }

  public void setElbowPosition(double degrees) {
    if (Math.abs(getElbowPosition().getDegrees() - degrees) > prefArm.elbowTolerance.getValue()) {
      double encoderCounts = SN_Math.degreesToFalcon(
          degrees, constArm.ELBOW_GEAR_RATIO);
      elbowJoint.set(ControlMode.Position, encoderCounts);
    } else {
      elbowJoint.neutralOutput();
    }
  }

  public void setShoulderPosition(SN_DoublePreference degrees) {
    setShoulderPosition(degrees.getValue());
  }

  public void setJointPercentOutputs(double shoulderPercent, double elbowPercent) {
    setShoulderPercentOutput(shoulderPercent);
    setElbowPercentOutput(elbowPercent);
  }

  public void setShoulderPercentOutput(double percent) {
    shoulderJoint.set(ControlMode.PercentOutput, percent * prefArm.shoulderMaxSpeed.getValue());
  }

  public void setElbowPercentOutput(double percent) {
    elbowJoint.set(ControlMode.PercentOutput, percent * prefArm.elbowMaxSpeed.getValue());
  }

  public Rotation2d getShoulderPosition() {
    double degrees = SN_Math.falconToDegrees(
        shoulderJoint.getSelectedSensorPosition(),
        constArm.SHOULDER_GEAR_RATIO);

    return Rotation2d.fromDegrees(degrees);
  }

  public Rotation2d getElbowPosition() {
    double degrees = SN_Math.falconToDegrees(
        elbowJoint.getSelectedSensorPosition(),
        constArm.ELBOW_GEAR_RATIO);

    return Rotation2d.fromDegrees(degrees);
  }

  public Rotation2d getShoulderAbsoluteEncoder() {
    double rotations = shoulderEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.SHOULDER_ABSOLUTE_ENCODER_OFFSET);
    return Rotation2d.fromRotations(rotations);
  }

  public Rotation2d getElbowAbsoluteEncoder() {
    double rotations = elbowEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.ELBOW_ABSOLUTE_ENCODER_OFFSET);
    return Rotation2d.fromRotations(rotations);
  }

  public void resetJointsToAbsolute() {
    resetShoulderToAbsolute();
    resetElbowToAbsolute();
  }

  private void resetShoulderToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getShoulderAbsoluteEncoder().getDegrees(),
        constArm.SHOULDER_GEAR_RATIO);
    shoulderJoint.setSelectedSensorPosition(absoluteEncoderCount);
  }

  private void resetElbowToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getElbowAbsoluteEncoder().getDegrees(),
        constArm.ELBOW_GEAR_RATIO);
    elbowJoint.setSelectedSensorPosition(absoluteEncoderCount);
  }

  private Translation2d getEEPosition() {
    double a1 = constArm.SHOULDER_LENGTH;
    double a2 = constArm.ELBOW_LENGTH;

    double q1 = getShoulderPosition().getRadians();
    double q2 = getElbowPosition().getRadians();

    double x = (a2 * Math.cos(q1 + q2)) + (a1 * Math.cos(q1));
    double y = (a2 * Math.sin(q1 + q2)) + (a1 * Math.sin(q1));

    return new Translation2d(x, y);
  }

  @Override
  public void periodic() {

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder Raw", shoulderEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder", getShoulderAbsoluteEncoder().getDegrees());
      SmartDashboard.putNumber("Arm Shoulder Motor Encoder Raw", shoulderJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Shoulder Position", getShoulderPosition().getDegrees());
      SmartDashboard.putNumber("Arm Shoulder Motor Output", shoulderJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw", elbowEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Elbow Absolute Encoder", getElbowAbsoluteEncoder().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Raw", elbowJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Elbow Position", getElbowPosition().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Output", elbowJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm EE Position X", Units.metersToInches(getEEPosition().getX()));
      SmartDashboard.putNumber("Arm EE Position Y", Units.metersToInches(getEEPosition().getY()));
      SmartDashboard.putNumber("Arm EE Distance",
          Units.metersToInches(Math.sqrt(Math.pow(getEEPosition().getX(), 2) + Math.pow(getEEPosition().getY(), 2))));
    }
  }
}
