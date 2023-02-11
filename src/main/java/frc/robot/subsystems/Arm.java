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

  Translation2d goalArmTipPosition;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);

    shoulderConfig = new TalonFXConfiguration();
    elbowConfig = new TalonFXConfiguration();

    goalArmTipPosition = new Translation2d();

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

    // general config
    resetJointsToAbsolute();
  }

  /**
   * Set the translational position of the tip of the arm (intake position). X is
   * horizontal, Y is vertical.
   * 
   * @param x x Position in meters
   * @param y y Position in meters
   */
  public void setArmTipPosition(Translation2d position) {

    double shoulderLength = constArm.SHOULDER_LENGTH;
    double elbowLength = constArm.ELBOW_LENGTH;

    double x2 = position.getX();
    double y2 = position.getY();

    // distance from from origin to arm tip goal position
    double R = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2));

    if (position.getDistance(new Translation2d()) < Units.inchesToMeters(prefArm.armTipDeadzone.getValue())) {
      System.err.println("Cannot input arm tip position within inner deadzone. Illegal values: X "
          + position.getX() + " Y: " + position.getY());
    }

    if (position.getDistance(new Translation2d()) > constArm.SHOULDER_LENGTH + constArm.ELBOW_LENGTH) {
      System.err.println(
          "Cannot input arm tip position that arm cannot reach. Make sure you are using the correct position for the arm tip. Illegal Values: "
              + position.getX() + " Y: " + position.getY());
    }

    // negative solution, i values are for x, j values are for y
    // https://www.desmos.com/calculator/kb6pranxy3
    double i1 = 1.0 / 2.0;
    double i2 = x2;
    double i3 = (Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2)) / (2 * Math.pow(R, 2));
    double i4 = x2;
    double i5 = 1.0 / 2.0;
    double i6 = 2 * ((Math.pow(shoulderLength, 2) + Math.pow(elbowLength, 2)) / Math.pow(R, 2));
    double i7 = Math.pow(Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2), 2) / Math.pow(R, 4);
    double i8 = 1;
    double i9 = y2;
    double ifinal = (i1 * i2) + (i3 * i4) - (i5 * Math.sqrt(i6 - i7 - i8) * i9);
    double elbowX = ifinal;

    double j1 = 1.0 / 2.0;
    double j2 = y2;
    double j3 = (Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2)) / (2 * Math.pow(R, 2));
    double j4 = y2;
    double j5 = 1.0 / 2.0;
    double j6 = 2 * ((Math.pow(shoulderLength, 2) + Math.pow(elbowLength, 2)) / Math.pow(R, 2));
    double j7 = Math.pow(Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2), 2) / Math.pow(R, 4);
    double j8 = 1;
    double j9 = -x2;
    double jfinal = (j1 * j2) + (j3 * j4) - (j5 * Math.sqrt(j6 - j7 - j8) * j9);
    double elbowY = jfinal;

    double shoulderAngle = Math.atan2(elbowY, elbowX);

    double elbowAngle = Math.atan2(y2 - elbowY, x2 - elbowX);

    setJointPositions(Units.radiansToDegrees(shoulderAngle), Units.radiansToDegrees(elbowAngle));

    if (Constants.OUTPUT_DEBUG_VALUES) {

      SmartDashboard.putNumber("Arm Debug Input Tip Position Meters (x2)", x2);
      SmartDashboard.putNumber("Arm Debug Input Tip Position Meters (y2)", y2);

      SmartDashboard.putNumber("Arm Debug shoulderLength meters", shoulderLength);
      SmartDashboard.putNumber("Arm Debug shoulderLength inches", Units.metersToInches(shoulderLength));

      SmartDashboard.putNumber("Arm Debug elbowLength meters", elbowLength);
      SmartDashboard.putNumber("Arm Debug elbowLength inches", Units.metersToInches(elbowLength));

      SmartDashboard.putNumber("Arm Debug Distance To Goal Tip Pose meters (R)", R);
      SmartDashboard.putNumber("Arm Debug R Distance To Goal Tip Pose inches (R)", Units.metersToInches(R));

      SmartDashboard.putNumber("Arm Debug i1", i1);
      SmartDashboard.putNumber("Arm Debug i2", i2);
      SmartDashboard.putNumber("Arm Debug i3", i3);
      SmartDashboard.putNumber("Arm Debug i4", i4);
      SmartDashboard.putNumber("Arm Debug i5", i5);
      SmartDashboard.putNumber("Arm Debug i6", i6);
      SmartDashboard.putNumber("Arm Debug i7", i7);
      SmartDashboard.putNumber("Arm Debug i8", i8);
      SmartDashboard.putNumber("Arm Debug i9", i9);
      SmartDashboard.putNumber("Arm Debug ifinal", ifinal);

      SmartDashboard.putNumber("Arm Debug j1", j1);
      SmartDashboard.putNumber("Arm Debug j2", j2);
      SmartDashboard.putNumber("Arm Debug j3", j3);
      SmartDashboard.putNumber("Arm Debug j4", j4);
      SmartDashboard.putNumber("Arm Debug j5", j5);
      SmartDashboard.putNumber("Arm Debug j6", j6);
      SmartDashboard.putNumber("Arm Debug j7", j7);
      SmartDashboard.putNumber("Arm Debug j8", j8);
      SmartDashboard.putNumber("Arm Debug j9", j9);
      SmartDashboard.putNumber("Arm Debug jfinal", jfinal);

      SmartDashboard.putNumber("Arm Debug elbowX meters", elbowX);
      SmartDashboard.putNumber("Arm Debug elbowX inches", Units.metersToInches(elbowX));

      SmartDashboard.putNumber("Arm Debug elbowY meters", elbowY);
      SmartDashboard.putNumber("Arm Debug elbowY inches", Units.metersToInches(elbowY));

      SmartDashboard.putNumber("Arm Debug shoulderAngle radians", shoulderAngle);
      SmartDashboard.putNumber("Arm Debug shoulderAngle degrees", Units.radiansToDegrees(shoulderAngle));

      SmartDashboard.putNumber("Arm Debug elbowAngle radians", elbowAngle);
      SmartDashboard.putNumber("Arm Debug elbowAngle degrees", Units.radiansToDegrees(elbowAngle));
    }
  }

  /**
   * Set the translational position of the tip of the arm (intake position).
   * 
   * @param x x Position in inches
   * @param y y Position in inches
   */
  public void setArmTipPositionInches(SN_DoublePreference x, SN_DoublePreference y) {
    setArmTipPosition(new Translation2d(Units.inchesToMeters(x.getValue()), Units.inchesToMeters(y.getValue())));
  }

  /**
   * Set the rotational positions of the shoulder and elbow joints.
   * 
   * @param shoulderAngle Shoulder position in degrees
   * @param elbowAngle    Elbow position in degrees
   */
  public void setJointPositions(double shoulderAngle, double elbowAngle) {
    setShoulderPosition(shoulderAngle);
    setElbowPosition(elbowAngle - shoulderAngle);
  }

  /**
   * Set the rotational positions of the shoulder and elbow joints.
   * 
   * @param shoulderAngle Shoulder position in degrees
   * @param elbowAngle    Elbow position in degrees
   */
  public void setJointPositions(SN_DoublePreference shoulderAngle, SN_DoublePreference elbowAngle) {
    setJointPositions(shoulderAngle.getValue(), elbowAngle.getValue());
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

  public Rotation2d getElbowPositionIndependent() {
    double degrees = SN_Math.falconToDegrees(
        elbowJoint.getSelectedSensorPosition(),
        constArm.ELBOW_GEAR_RATIO);

    return Rotation2d.fromDegrees(degrees + getShoulderPosition().getDegrees());
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
  private Translation2d getArmTipPosition() {
    double a1 = constArm.SHOULDER_LENGTH;
    double a2 = constArm.ELBOW_LENGTH;

    double q1 = getShoulderPosition().getRadians();
    double q2 = getElbowPosition().getRadians();

    double x = (a2 * Math.cos(q1 + q2)) + (a1 * Math.cos(q1));
    double y = (a2 * Math.sin(q1 + q2)) + (a1 * Math.sin(q1));

    return new Translation2d(x, y);
  }

  /**
   * Set the goal arm tip position. This will not move the arm on its own.
   * 
   * @param goalPosition Goal arm tip position in meters
   */
  public void setGoalArmTipPosition(Translation2d goalPosition) {
    goalArmTipPosition = goalPosition;
  }

  /**
   * Get the goal arm tip position.
   * 
   * @return The goal arm tip position in meters.
   */
  public Translation2d getGoalArmTipPosition() {
    return goalArmTipPosition;
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
      SmartDashboard.putNumber("Arm Elbow Position Independent", getElbowPositionIndependent().getDegrees());

      SmartDashboard.putNumber("Arm Tip Position X", Units.metersToInches(getArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Tip Position Y", Units.metersToInches(getArmTipPosition().getY()));
      SmartDashboard.putNumber("Arm Tip Distance",
          Units.metersToInches(getArmTipPosition().getDistance(new Translation2d())));

      SmartDashboard.putNumber("Arm Goal Tip Position X", Units.metersToInches(getGoalArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Goal Tip Position Y", Units.metersToInches(getGoalArmTipPosition().getY()));
    }
  }
}
