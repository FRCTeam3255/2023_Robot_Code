// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.constArm;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotMap.mapArm;
import frc.robot.RobotPreferences.prefArm;

public class Arm extends SubsystemBase {

  TalonFX shoulderJoint;
  TalonFX elbowJoint;

  TalonFXConfiguration shoulderConfig;
  TalonFXConfiguration elbowConfig;

  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder elbowEncoder;

  double shoulderOffset;
  double elbowOffset;

  public Arm() {
    shoulderJoint = new TalonFX(mapArm.SHOULDER_CAN);
    elbowJoint = new TalonFX(mapArm.ELBOW_CAN);

    shoulderConfig = new TalonFXConfiguration();
    elbowConfig = new TalonFXConfiguration();

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);

    if (RobotContainer.isPracticeBot()) {
      shoulderOffset = constArm.PRAC_SHOULDER_ABSOLUTE_ENCODER_OFFSET;
      elbowOffset = constArm.PRAC_ELBOW_ABSOLUTE_ENCODER_OFFSET;
    } else {
      shoulderOffset = constArm.SHOULDER_ABSOLUTE_ENCODER_OFFSET;
      elbowOffset = constArm.ELBOW_ABSOLUTE_ENCODER_OFFSET;
    }

    configure();
  }

  public void configure() {

    // shoulder config
    shoulderJoint.configFactoryDefault();

    shoulderConfig.slot0.kP = prefArm.shoulderP.getValue();
    shoulderConfig.slot0.kI = prefArm.shoulderI.getValue();
    shoulderConfig.slot0.kD = prefArm.shoulderD.getValue();

    shoulderConfig.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(
        prefArm.shoulderTolerance.getValue(),
        constArm.SHOULDER_GEAR_RATIO);

    shoulderConfig.slot0.closedLoopPeakOutput = prefArm.shoulderClosedLoopPeakOutput.getValue();

    shoulderConfig.forwardSoftLimitEnable = true;
    shoulderConfig.reverseSoftLimitEnable = true;

    shoulderConfig.forwardSoftLimitThreshold = SN_Math
        .degreesToFalcon(Units.radiansToDegrees(constArm.SHOULDER_FORWARD_LIMIT), constArm.SHOULDER_GEAR_RATIO);
    shoulderConfig.reverseSoftLimitThreshold = SN_Math
        .degreesToFalcon(Units.radiansToDegrees(constArm.SHOULDER_REVERSE_LIMIT), constArm.SHOULDER_GEAR_RATIO);

    shoulderJoint.configAllSettings(shoulderConfig);

    shoulderJoint.setInverted(constArm.SHOULDER_MOTOR_INVERT);
    shoulderJoint.setNeutralMode(constArm.SHOULDER_MOTOR_BREAK);

    // elbow config
    elbowJoint.configFactoryDefault();

    elbowConfig.slot0.kP = prefArm.elbowP.getValue();
    elbowConfig.slot0.kI = prefArm.elbowI.getValue();
    elbowConfig.slot0.kD = prefArm.elbowD.getValue();

    elbowConfig.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(
        prefArm.elbowTolerance.getValue(),
        constArm.ELBOW_GEAR_RATIO);

    elbowConfig.slot0.closedLoopPeakOutput = prefArm.elbowClosedLoopPeakOutput.getValue();

    elbowConfig.forwardSoftLimitEnable = true;
    elbowConfig.reverseSoftLimitEnable = true;

    elbowConfig.forwardSoftLimitThreshold = SN_Math
        .degreesToFalcon(Units.radiansToDegrees(constArm.ELBOW_FORWARD_LIMIT), constArm.ELBOW_GEAR_RATIO);
    elbowConfig.reverseSoftLimitThreshold = SN_Math
        .degreesToFalcon(Units.radiansToDegrees(constArm.ELBOW_REVERSE_LIMIT), constArm.ELBOW_GEAR_RATIO);

    elbowJoint.configAllSettings(elbowConfig);

    elbowJoint.setInverted(constArm.ELBOW_MOTOR_INVERT);
    elbowJoint.setNeutralMode(constArm.ELBOW_MOTOR_BREAK);
  }

  public void resetJointEncodersToAbsolute() {
    shoulderJoint.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(getShoulderAbsoluteEncoder().getDegrees(), constArm.SHOULDER_GEAR_RATIO));

    elbowJoint.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(getElbowAbsoluteEncoder().getDegrees(), constArm.ELBOW_GEAR_RATIO));
  }

  public void setJointsNeutralMode() {
    shoulderJoint.setNeutralMode(constArm.SHOULDER_MOTOR_BREAK);
    elbowJoint.setNeutralMode(constArm.ELBOW_MOTOR_BREAK);
  }

  /**
   * Set the rotational positions of the shoulder and elbow joints.
   * 
   * @param shoulderAngle Shoulder position
   * @param elbowAngle    Elbow position
   */
  private void setJointPositions(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
    double shoulderCounts = SN_Math.degreesToFalcon(shoulderAngle.getDegrees(), constArm.SHOULDER_GEAR_RATIO);
    shoulderJoint.set(ControlMode.Position, shoulderCounts);

    double elbowCounts = SN_Math.degreesToFalcon(elbowAngle.getDegrees(), constArm.ELBOW_GEAR_RATIO);
    elbowJoint.set(ControlMode.Position, elbowCounts);
  }

  /**
   * Set the percent output of the shoulder and elbow joint.
   * 
   * @param shoulderPercent Shoulder percent output
   * @param elbowPercent    Elbow percent output
   */
  private void setJointPercentOutputs(double shoulderPercent, double elbowPercent) {
    shoulderJoint.set(ControlMode.PercentOutput, shoulderPercent);
    elbowJoint.set(ControlMode.PercentOutput, elbowPercent);
  }

  /**
   * Neutral the outputs of each joint motor
   */
  public void neutralJointOutputs() {
    shoulderJoint.neutralOutput();
    elbowJoint.neutralOutput();
  }

  /**
   * Get the shoulder absolute encoder reading.
   * 
   * @return Shoulder absolute encoder reading
   */
  private Rotation2d getShoulderAbsoluteEncoder() {
    double rotations = shoulderEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(shoulderOffset);
    rotations = MathUtil.inputModulus(rotations, -0.5, 0.5);

    if (constArm.SHOULDER_ABSOLUTE_ENCODER_INVERT) {
      return Rotation2d.fromRotations(-rotations);
    } else {
      return Rotation2d.fromRotations(rotations);
    }
  }

  /**
   * Get the elbow absolute encoder reading.
   * 
   * @return Elbow absolute encoder reading
   */
  private Rotation2d getElbowAbsoluteEncoder() {
    double rotations = elbowEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(elbowOffset);
    rotations = MathUtil.inputModulus(rotations, -0.5, 0.5);

    if (constArm.ELBOW_ABSOLUTE_ENCODER_INVERT) {
      return Rotation2d.fromRotations(-rotations);
    } else {
      return Rotation2d.fromRotations(rotations);
    }
  }

  /**
   * Get the shoulder joint position from the motor.
   * 
   * @return Shoulder joint position
   */
  private Rotation2d getShoulderPosition() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(shoulderJoint.getSelectedSensorPosition(), constArm.SHOULDER_GEAR_RATIO));
  }

  /**
   * Get the elbow joint position from the motor.
   * 
   * @return Elbow joint position
   */
  private Rotation2d getElbowPosition() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(elbowJoint.getSelectedSensorPosition(), constArm.ELBOW_GEAR_RATIO));
  }

  /**
   * Get the current state of the arm. If the arm is not currently at a valid
   * state, this will return ArmState.NONE.
   * 
   * @return Current arm state.
   */
  public ArmState getCurrentState() {
    for (ArmState state : ArmState.values()) {
      if (areJointsInToleranceToState(state)) {
        return state;
      }
    }

    return ArmState.NONE;
  }

  private boolean isJointInToleranceToAngle(Rotation2d jointRotation, Rotation2d goalRotation, Rotation2d tolerance) {
    double jointToGoal = Math.abs(jointRotation.getRadians() - goalRotation.getDegrees());
    double fudgedTolerance = tolerance.getRadians() * prefArm.armToleranceFudgeFactor.getValue();

    return jointToGoal < fudgedTolerance;
  }

  private boolean areJointsInToleranceToState(ArmState state) {
    boolean isShoulderInTolerance = isJointInToleranceToAngle(
        getShoulderPosition(),
        state.shoulderAngle,
        Rotation2d.fromDegrees(prefArm.shoulderTolerance.getValue()));

    boolean isElbowInTolerance = isJointInToleranceToAngle(
        getElbowPosition(),
        state.elbowAngle,
        Rotation2d.fromDegrees(prefArm.elbowTolerance.getValue()));

    return isShoulderInTolerance && isElbowInTolerance;
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
    double q2 = getElbowPosition().getRadians() + q1;

    double x = (a2 * Math.cos(q1 + q2)) + (a1 * Math.cos(q1));
    double y = (a2 * Math.sin(q1 + q2)) + (a1 * Math.sin(q1));

    return new Translation2d(x, y);
  }

  @Override
  public void periodic() {

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder Raw", shoulderEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Shoulder Motor Encoder Raw", shoulderJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Shoulder Position", getShoulderPosition().getDegrees());
      SmartDashboard.putNumber("Arm Shoulder Motor Output", shoulderJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw", elbowEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Raw", elbowJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Elbow Position", getElbowPosition().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Output", elbowJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Tip Position X", Units.metersToInches(getArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Tip Position Y", Units.metersToInches(getArmTipPosition().getY()));
      SmartDashboard.putNumber("Arm Tip Distance",
          Units.metersToInches(getArmTipPosition().getDistance(new Translation2d())));

      SmartDashboard.putNumber("Arm PID Shoulder Goal",
          SN_Math.falconToDegrees(shoulderJoint.getClosedLoopTarget(), constArm.SHOULDER_GEAR_RATIO));
      SmartDashboard.putNumber("Arm PID Shoudler Error",
          SN_Math.falconToDegrees(shoulderJoint.getClosedLoopError(), constArm.SHOULDER_GEAR_RATIO));

      SmartDashboard.putNumber("Arm PID Elbow Goal",
          SN_Math.falconToDegrees(elbowJoint.getClosedLoopTarget(), constArm.ELBOW_GEAR_RATIO));
      SmartDashboard.putNumber("Arm PID Elbow Error",
          SN_Math.falconToDegrees(elbowJoint.getClosedLoopError(), constArm.ELBOW_GEAR_RATIO));
    }
  }
}
