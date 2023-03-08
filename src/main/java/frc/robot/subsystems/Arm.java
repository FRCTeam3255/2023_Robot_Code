// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
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
import frc.robot.Constants.constControllers.ScoringButton;
import frc.robot.Constants.constControllers.ScoringGrid;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.Constants.constVision.GamePiece;
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

  Rotation2d goalShoulderAngle;
  Rotation2d goalElbowAngle;

  public GamePiece desiredGamePiece = GamePiece.NONE;
  public ScoringLevel scoringLevel = ScoringLevel.NONE;
  public ScoringButton scoringButton = ScoringButton.NONE;
  public ScoringGrid scoringGrid = ScoringGrid.NONE;

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

    goalShoulderAngle = new Rotation2d();
    goalElbowAngle = new Rotation2d();

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
    resetShoulderJointToAbsolute();
    resetElbowJointToAbsolute();
  }

  private void resetShoulderJointToAbsolute() {
    shoulderJoint.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(getShoulderAbsoluteEncoder().getDegrees(), constArm.SHOULDER_GEAR_RATIO));
  }

  private void resetElbowJointToAbsolute() {
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
  public void setJointPositions(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
    setShoulderPosition(shoulderAngle);
    setElbowPosition(elbowAngle);
  }

  /**
   * Set the rotational position of the shoulder joint.
   * 
   * @param position Rotational position to set shoulder
   */
  private void setShoulderPosition(Rotation2d position) {
    double counts = SN_Math.degreesToFalcon(position.getDegrees(), constArm.SHOULDER_GEAR_RATIO);

    shoulderJoint.set(ControlMode.Position, counts);
  }

  /**
   * Set the rotational position of the elbow joint.
   * 
   * @param degrees Rotational position to set elbow
   */
  private void setElbowPosition(Rotation2d position) {
    double counts = SN_Math.degreesToFalcon(position.getDegrees(), constArm.ELBOW_GEAR_RATIO);

    elbowJoint.set(ControlMode.Position, counts);
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
    shoulderJoint.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Set the percent output of the elbow joint motor.
   * 
   * @param percent Percent output to set
   */
  public void setElbowPercentOutput(double percent) {
    elbowJoint.set(ControlMode.PercentOutput, percent);
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
  public Rotation2d getShoulderAbsoluteEncoder() {
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
  public Rotation2d getElbowAbsoluteEncoder() {
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
  public Rotation2d getShoulderPosition() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(shoulderJoint.getSelectedSensorPosition(), constArm.SHOULDER_GEAR_RATIO));
  }

  /**
   * Get the elbow joint position from the motor.
   * 
   * @return Elbow joint position
   */
  public Rotation2d getElbowPosition() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(elbowJoint.getSelectedSensorPosition(), constArm.ELBOW_GEAR_RATIO));
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

  public boolean isShoulderInTolerance() {
    return Math.abs(getShoulderPosition().getRadians() - getGoalShoulderAngle().getRadians()) < (Units
        .degreesToRadians(prefArm.shoulderTolerance.getValue() * prefArm.armToleranceFudgeFactor.getValue()));
  }

  public boolean isElbowInTolerance() {
    return Math.abs(getElbowPosition().getRadians() - getGoalElbowAngle().getRadians()) < Units
        .degreesToRadians(prefArm.elbowTolerance.getValue() * prefArm.armToleranceFudgeFactor.getValue());
  }

  public boolean areJointsInTolerance() {
    return isShoulderInTolerance() && isElbowInTolerance();
  }

  public boolean isCubeNode() {
    if (scoringButton == ScoringButton.FIFTH || scoringButton == ScoringButton.EIGHTH) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isConeNode() {
    if (scoringButton == ScoringButton.FOURTH || scoringButton == ScoringButton.SIXTH
        || scoringButton == ScoringButton.SEVENTH || scoringButton == ScoringButton.NINTH) {
      return true;
    } else {
      return false;
    }
  }

  public void setGoalAnglesFromNumpad() {
    if (isCubeNode()) {
      if (scoringLevel == ScoringLevel.MID) {
        setGoalAngles(prefArm.armPresetCubeMidShoulderAngle, prefArm.armPresetCubeMidElbowAngle);
      } else if (scoringLevel == ScoringLevel.HIGH) {
        setGoalAngles(prefArm.armPresetCubeHighShoulderAngle, prefArm.armPresetCubeHighElbowAngle);
      } else if (scoringLevel == ScoringLevel.HYBRID) {
        setGoalAngles(prefArm.armPresetLowShoulderAngle, prefArm.armPresetLowElbowAngle);
      } else if (scoringLevel == ScoringLevel.NONE) {
        // do nothing in this case
      }

    } else if (isConeNode()) {
      if (scoringLevel == ScoringLevel.MID) {
        setGoalAngles(prefArm.armPresetConeMidShoulderAngle, prefArm.armPresetConeMidElbowAngle);
      } else if (scoringLevel == ScoringLevel.HIGH) {
        setGoalAngles(prefArm.armPresetConeHighShoulderAngle, prefArm.armPresetConeHighElbowAngle);
      } else if (scoringLevel == ScoringLevel.HYBRID) {
        setGoalAngles(prefArm.armPresetLowShoulderAngle, prefArm.armPresetLowElbowAngle);
      } else if (scoringLevel == ScoringLevel.NONE) {
        // do nothing in this case
      }

    } else {
      setGoalAngles(prefArm.armPresetLowShoulderAngle, prefArm.armPresetLowElbowAngle);
    }
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("desiredGamePiece", desiredGamePiece.toString());
    SmartDashboard.putString("scoringLevel", scoringLevel.toString());
    SmartDashboard.putString("scoringColumn", scoringButton.toString());

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

      SmartDashboard.putNumber("Arm Goal Angle Shoulder", goalShoulderAngle.getDegrees());
      SmartDashboard.putNumber("Arm Goal Angle Elbow", goalElbowAngle.getDegrees());

      SmartDashboard.putNumber("Arm PID Shoulder Goal",
          SN_Math.falconToDegrees(shoulderJoint.getClosedLoopTarget(), constArm.SHOULDER_GEAR_RATIO));
      SmartDashboard.putNumber("Arm PID Shoudler Error",
          SN_Math.falconToDegrees(shoulderJoint.getClosedLoopError(), constArm.SHOULDER_GEAR_RATIO));
      SmartDashboard.putBoolean("Arm PID Shoulder Is Within Tolerance", isShoulderInTolerance());

      SmartDashboard.putNumber("Arm PID Elbow Goal",
          SN_Math.falconToDegrees(elbowJoint.getClosedLoopTarget(), constArm.ELBOW_GEAR_RATIO));
      SmartDashboard.putNumber("Arm PID Elbow Error",
          SN_Math.falconToDegrees(elbowJoint.getClosedLoopError(), constArm.ELBOW_GEAR_RATIO));
      SmartDashboard.putBoolean("Arm PID Elbow Is Within Tolerance", isElbowInTolerance());

      SmartDashboard.putBoolean("Arm PID Joints Are Within Tolerance", areJointsInTolerance());

    }
  }
}
