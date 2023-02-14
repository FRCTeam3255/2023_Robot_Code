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
import edu.wpi.first.wpilibj.Timer;
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

  Rotation2d goalShoulderAngle;
  Rotation2d goalElbowAngle;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);

    shoulderConfig = new TalonFXConfiguration();
    elbowConfig = new TalonFXConfiguration();

    goalShoulderAngle = new Rotation2d();
    goalElbowAngle = new Rotation2d();

    configure();

    Timer.delay(2.25);
    resetJointsToAbsolute();
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

    shoulderConfig.forwardSoftLimitEnable = prefArm.shoulderForwardSoftLimit.getValue();
    shoulderConfig.reverseSoftLimitEnable = prefArm.shoulderReverseSoftLimit.getValue();

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

    elbowConfig.forwardSoftLimitEnable = prefArm.elbowForwardSoftLimit.getValue();
    elbowConfig.reverseSoftLimitEnable = prefArm.elbowReverseSoftLimit.getValue();

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
  }

  public void setJointsNeutralMode() {
    shoulderJoint.setNeutralMode(constArm.SHOULDER_MOTOR_BREAK);
    elbowJoint.setNeutralMode(constArm.ELBOW_MOTOR_BREAK);
  }

  /**
   * Set the rotational positions of the shoulder and elbow joints.
   * 
   * @param shoulderAngle Shoulder position in degrees
   * @param elbowAngle    Elbow position in degrees
   */
  public void setJointPositions(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
    setShoulderPosition(shoulderAngle.getDegrees());
    setElbowPosition(elbowAngle.getDegrees());
  }

  /**
   * Set the rotational positions of the shoulder and elbow joints.
   * 
   * @param shoulderAngle Shoulder position in degrees
   * @param elbowAngle    Elbow position in degrees
   */
  public void setJointPositions(SN_DoublePreference shoulderAngle, SN_DoublePreference elbowAngle) {
    setJointPositions(Rotation2d.fromDegrees(shoulderAngle.getValue()), Rotation2d.fromDegrees(elbowAngle.getValue()));
  }

  /**
   * Set the rotational position of the shoulder joint.
   * 
   * @param degrees Rotational position to set shoulder
   */
  public void setShoulderPosition(double degrees) {
    if (Math.abs(getShoulderPosition().getDegrees() - degrees) > prefArm.shoulderTolerance.getValue()) {
      double encoderCounts = SN_Math.degreesToFalcon(
          degrees, constArm.SHOULDER_GEAR_RATIO);
      shoulderJoint.set(ControlMode.Position, encoderCounts);
    } else {
      shoulderJoint.neutralOutput();
    }
  }

  /**
   * Set the rotational position of the elbow joint.
   * 
   * @param degrees Rotational position to set elbow
   */
  public void setElbowPosition(double degrees) {
    if (Math.abs(getElbowPosition().getDegrees() - degrees) > prefArm.elbowTolerance.getValue()) {
      double encoderCounts = SN_Math.degreesToFalcon(
          degrees, constArm.ELBOW_GEAR_RATIO);
      elbowJoint.set(ControlMode.Position, encoderCounts);
    } else {
      elbowJoint.neutralOutput();
    }
  }

  /**
   * Set the rotational position of the shoulder joint.
   * 
   * @param degrees Rotational position to set shoulder
   */
  public void setShoulderPosition(SN_DoublePreference degrees) {
    setShoulderPosition(degrees.getValue());
  }

  /**
   * Set the percent output of the shoulder and elbow joint.
   * 
   * @param shoulderPercent Shoulder percent output
   * @param elbowPercent    Elbow percent output
   */
  public void setJointPercentOutputs(double shoulderPercent, double elbowPercent) {
    setShoulderPercentOutput(shoulderPercent);
    setElbowPercentOutput(elbowPercent);
  }

  /**
   * Set the percent output of the shoulder joint motor.
   * 
   * @param percent Percent output to set
   */
  public void setShoulderPercentOutput(double percent) {
    shoulderJoint.set(ControlMode.PercentOutput, percent * prefArm.shoulderMaxSpeed.getValue());
  }

  /**
   * Set the percent output of the elbow joint motor.
   * 
   * @param percent Percent output to set
   */
  public void setElbowPercentOutput(double percent) {
    elbowJoint.set(ControlMode.PercentOutput, percent * prefArm.elbowMaxSpeed.getValue());
  }

  /**
   * Neutral the outputs of each joint motor
   */
  public void neutralJointOutputs() {
    shoulderJoint.neutralOutput();
    elbowJoint.neutralOutput();
  }

  /**
   * Get the rotational position of the shoulder.
   * 
   * @return Rotational position of shoulder
   */
  public Rotation2d getShoulderPosition() {
    double degrees = SN_Math.falconToDegrees(
        shoulderJoint.getSelectedSensorPosition(),
        constArm.SHOULDER_GEAR_RATIO);

    return Rotation2d.fromDegrees(degrees);
  }

  /**
   * Get the rotational position of the elbow.
   * 
   * @return Rotational position of elbow
   */
  public Rotation2d getElbowPosition() {
    double degrees = SN_Math.falconToDegrees(
        elbowJoint.getSelectedSensorPosition(),
        constArm.ELBOW_GEAR_RATIO);

    return Rotation2d.fromDegrees(degrees);
  }

  /**
   * Get the shoulder absolute encoder reading.
   * 
   * @return Shoulder absolute encoder reading
   */
  public Rotation2d getShoulderAbsoluteEncoder() {
    double rotations = shoulderEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.SHOULDER_ABSOLUTE_ENCODER_OFFSET);
    return Rotation2d.fromRotations(rotations);
  }

  /**
   * Get the elbow absolute encoder reading.
   * 
   * @return Elbow absolute encoder reading
   */
  public Rotation2d getElbowAbsoluteEncoder() {
    double rotations = elbowEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.ELBOW_ABSOLUTE_ENCODER_OFFSET);
    rotations %= 1.0;
    return Rotation2d.fromRotations(rotations);
  }

  /**
   * Reset the shoulder and elbow motor encoder to the respective absolute
   * encoders.
   */
  public void resetJointsToAbsolute() {
    resetShoulderToAbsolute();
    resetElbowToAbsolute();
  }

  /**
   * Reset the shoulder motor encoder to the absolute shoulder encoder.
   */
  private void resetShoulderToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getShoulderAbsoluteEncoder().getDegrees(),
        constArm.SHOULDER_GEAR_RATIO);
    shoulderJoint.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Reset the elbow motor encoder to the absolute elbow encoder.
   */
  private void resetElbowToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getElbowAbsoluteEncoder().getDegrees(),
        constArm.ELBOW_GEAR_RATIO);
    elbowJoint.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Get the position of the arm tip in 2D space relative to the robot in meters.
   * 
   * @return Position of of arm tip in meters
   */
  public Translation2d getArmTipPosition() {
    double a1 = constArm.SHOULDER_LENGTH;
    double a2 = constArm.ELBOW_LENGTH;

    double q1 = getShoulderPosition().getRadians();
    double q2 = getElbowPosition().getRadians() + q1;

    double x = (a2 * Math.cos(q1 + q2)) + (a1 * Math.cos(q1));
    double y = (a2 * Math.sin(q1 + q2)) + (a1 * Math.sin(q1));

    return new Translation2d(x, y);
  }

  /**
   * Set the goal angles for the shoulder and elbow.
   * 
   * @param shoulderAngle Goal shoulder angle
   * @param elbowAngle    Goal elbow angle
   */
  public void setGoalAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
    goalShoulderAngle = shoulderAngle;
    goalElbowAngle = elbowAngle;
  }

  /**
   * Get the goal shoulder angle.
   * 
   * @return Goal shoulder angle
   */
  public Rotation2d getGoalShoulderAngle() {
    return goalShoulderAngle;
  }

  /**
   * Get the goal elbow angle.
   * 
   * @return Goal elbow angle.
   */
  public Rotation2d getGoalElbowAngle() {
    return goalElbowAngle;
  }

  /**
   * Set the goal angles for the shoulder and elbow using SN_DoublePreferences.
   * 
   * @param shoulderDegrees Shoulder goal angle in degrees
   * @param elbowDegrees    Elbow goal angle in degrees
   */
  public void setGoalAngles(SN_DoublePreference shoulderDegrees, SN_DoublePreference elbowDegrees) {
    setGoalAngles(Rotation2d.fromDegrees(shoulderDegrees.getValue()), Rotation2d.fromDegrees(elbowDegrees.getValue()));
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
      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw Degrees",
          Units.rotationsToDegrees(elbowEncoder.getAbsolutePosition()));
      SmartDashboard.putNumber("Arm Elbow Absolute Encoder", getElbowAbsoluteEncoder().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Raw", elbowJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Elbow Position", getElbowPosition().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Output", elbowJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Tip Position X", Units.metersToInches(getArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Tip Position Y", Units.metersToInches(getArmTipPosition().getY()));
      SmartDashboard.putNumber("Arm Tip Distance",
          Units.metersToInches(getArmTipPosition().getDistance(new Translation2d())));

      SmartDashboard.putNumber("Arm Goal Angle Elbow", goalElbowAngle.getDegrees());
      SmartDashboard.putNumber("Arm Goal Angle Shoulder", goalShoulderAngle.getDegrees());
    }
  }
}
