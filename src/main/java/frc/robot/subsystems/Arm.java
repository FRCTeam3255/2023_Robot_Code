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

    shoulderConfig.forwardSoftLimitEnable = true;
    shoulderConfig.reverseSoftLimitEnable = true;

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

    elbowConfig.forwardSoftLimitEnable = true;
    elbowConfig.reverseSoftLimitEnable = true;

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
   * Set the translational position of the tip of the arm (intake position).
   * 
   * @param x x Position in inches
   * @param y y Position in inches
   */
  public void setArmTipPosition(Translation2d position) {

    SmartDashboard.putNumber("Arm Debug input tip position inches x", position.getX());
    SmartDashboard.putNumber("Arm Debug input tip position inches y", position.getY());

    if (position.getDistance(new Translation2d()) < Units.inchesToMeters(prefArm.armTipDeadzone.getValue())) {
      System.out.println("Cannot input arm tip position within inner deadzone. Illegal values: X " + position.getX()
          + " Y: " + position.getY());
      System.out.println(3255 / 0);
    }

    if (position.getDistance(new Translation2d()) > Units.metersToInches(constArm.SHOULDER_LENGTH)
        + Units.metersToInches(constArm.ELBOW_LENGTH)) {
      System.out.println(
          "Cannot input arm tip position that arm cannot reach. Make sure you are using the correct position for the arm tip. Illegal Values: "
              + position.getX() + " Y: " + position.getY());
    }

    double x = Units.inchesToMeters(position.getX());
    double y = Units.inchesToMeters(position.getY());

    double shoulderLength = constArm.SHOULDER_LENGTH;
    double elbowLength = constArm.ELBOW_LENGTH;

    // distance from from origin to arm tip goal position
    double R = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    SmartDashboard.putNumber("Arm Debug R (distance to goal tip pose) meters", R);
    SmartDashboard.putNumber("Arm Debug R (distance to goal tip pose) inches", Units.metersToInches(R));

    // negative solution

    // see https://www.desmos.com/calculator/rxoywnwrcg for math (lines 15 and 16)
    double elbowX = (1 / 2) * (x)
        + ((Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2)) / (2 * Math.pow(R, 2))) * (x)
        - (1 / 2) * Math.sqrt(2 * ((Math.pow(shoulderLength, 2) + Math.pow(elbowLength, 2)) / Math.pow(R, 2))
            - (Math.pow((Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2)), 2) / Math.pow(R, 4)) - 1) * (y);

    SmartDashboard.putNumber("Arm Debug elbowX meters", elbowX);
    SmartDashboard.putNumber("Arm Debug elbowX inches", Units.metersToInches(elbowX));

    double elbowY = (1 / 2) * (y)
        + ((Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2)) / (2 * Math.pow(R, 2))) * (y)
        - (1 / 2) * Math.sqrt(2 * ((Math.pow(shoulderLength, 2) + Math.pow(elbowLength, 2)) / Math.pow(R, 2))
            - (Math.pow(Math.pow(shoulderLength, 2) - Math.pow(elbowLength, 2), 2) / Math.pow(R, 4)) - 1) * (x);

    SmartDashboard.putNumber("Arm Debug elbowY meters", elbowY);
    SmartDashboard.putNumber("Arm Debug elbowY inches", Units.metersToInches(elbowY));

    double shoulderAngle = Math.atan2(elbowY, elbowX);

    SmartDashboard.putNumber("Arm Debug shoulderAngle radians", shoulderAngle);
    SmartDashboard.putNumber("Arm Debug shoulderAngle degrees", Units.radiansToDegrees(shoulderAngle));

    double elbowAngle = Math.atan2(y - elbowY, x - elbowX);

    SmartDashboard.putNumber("Arm Debug elbowAngle radians", elbowAngle);
    SmartDashboard.putNumber("Arm Debug elbowAngle degrees", Units.radiansToDegrees(elbowAngle));

    setJointPositions(Units.radiansToDegrees(shoulderAngle), Units.radiansToDegrees(elbowAngle));
  }

  /**
   * Set the translational position of the tip of the arm (intake position).
   * 
   * @param x x Position in inches
   * @param y y Position in inches
   */
  public void setArmTipPosition(SN_DoublePreference x, SN_DoublePreference y) {
    setArmTipPosition(new Translation2d(x.getValue(), y.getValue()));
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

  @Override
  public void periodic() {

    setArmTipPosition(new Translation2d(8.0, 8.0));

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

      SmartDashboard.putNumber("Arm Tip Position X", Units.metersToInches(getArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Tip Position Y", Units.metersToInches(getArmTipPosition().getY()));
      SmartDashboard.putNumber("Arm Tip Distance",
          Units.metersToInches(getArmTipPosition().getDistance(new Translation2d())));
    }
  }
}
