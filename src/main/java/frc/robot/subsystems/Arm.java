// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constArm;
import frc.robot.Constants.constControllers.ScoringColumn;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.Constants.constVision.GamePiece;
import frc.robot.RobotMap.mapArm;
import frc.robot.RobotPreferences.prefArm;

public class Arm extends SubsystemBase {

  SN_CANSparkMax shoulderJoint;
  SN_CANSparkMax elbowJoint;

  DutyCycleEncoder shoulderEncoder;
  DutyCycleEncoder elbowEncoder;

  Rotation2d goalShoulderAngle;
  Rotation2d goalElbowAngle;

  ProfiledPIDController shoulderPID;
  ProfiledPIDController elbowPID;

  public GamePiece desiredGamePiece = GamePiece.NONE;
  public ScoringLevel scoringLevel = ScoringLevel.NONE;
  public ScoringColumn scoringColumn = ScoringColumn.NONE;

  public Arm() {
    shoulderJoint = new SN_CANSparkMax(mapArm.SHOULDER_CAN);
    elbowJoint = new SN_CANSparkMax(mapArm.ELBOW_CAN);

    shoulderEncoder = new DutyCycleEncoder(mapArm.SHOULDER_ABSOLUTE_ENCODER_DIO);
    elbowEncoder = new DutyCycleEncoder(mapArm.ELBOW_ABSOLUTE_ENCODER_DIO);

    goalShoulderAngle = new Rotation2d();
    goalElbowAngle = new Rotation2d();

    shoulderPID = new ProfiledPIDController(
        prefArm.shoulderP.getValue(),
        prefArm.shoulderI.getValue(),
        prefArm.shoulderD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefArm.shoulderMaxSpeed.getValue()),
            Units.degreesToRadians(prefArm.shoulderMaxAccel.getValue())));

    elbowPID = new ProfiledPIDController(
        prefArm.elbowP.getValue(),
        prefArm.elbowI.getValue(),
        prefArm.elbowD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefArm.elbowMaxSpeed.getValue()),
            Units.degreesToRadians(prefArm.elbowMaxAccel.getValue())));

    configure();
  }

  public void configure() {

    // shoulder config
    shoulderJoint.configFactoryDefault();

    shoulderJoint.setInverted(constArm.SHOULDER_MOTOR_INVERT);
    shoulderJoint.setNeutralMode(constArm.SHOULDER_MOTOR_BREAK);

    shoulderPID.setPID(
        prefArm.shoulderP.getValue(),
        prefArm.shoulderI.getValue(),
        prefArm.shoulderD.getValue());
    shoulderPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefArm.shoulderMaxSpeed.getValue()),
            Units.degreesToRadians(prefArm.shoulderMaxAccel.getValue())));
    shoulderPID.setTolerance(Units.degreesToRadians(prefArm.shoulderTolerance.getValue()));

    // elbow config
    elbowJoint.configFactoryDefault();

    elbowJoint.setInverted(constArm.ELBOW_MOTOR_INVERT);
    elbowJoint.setNeutralMode(constArm.ELBOW_MOTOR_BREAK);

    elbowPID.setPID(
        prefArm.elbowP.getValue(),
        prefArm.elbowI.getValue(),
        prefArm.elbowD.getValue());
    elbowPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefArm.elbowMaxSpeed.getValue()),
            Units.degreesToRadians(prefArm.elbowMaxAccel.getValue())));
    elbowPID.setTolerance(Units.degreesToRadians(prefArm.elbowTolerance.getValue()));
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
    double radians = MathUtil.clamp(
        position.getRadians(),
        constArm.SHOULDER_REVERSE_LIMIT,
        constArm.SHOULDER_FORWARD_LIMIT);
    shoulderPID.setGoal(radians);

    setShoulderPercentOutput(shoulderPID.calculate(getShoulderPosition().getRadians()));
  }

  /**
   * Set the rotational position of the elbow joint.
   * 
   * @param degrees Rotational position to set elbow
   */
  private void setElbowPosition(Rotation2d position) {
    double radians = MathUtil.clamp(
        position.getRadians(),
        constArm.ELBOW_REVERSE_LIMIT,
        constArm.ELBOW_FORWARD_LIMIT);
    elbowPID.setGoal(radians);

    setElbowPercentOutput(elbowPID.calculate(getElbowPosition().getRadians()));
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
    double output = percent;

    if (getShoulderPosition().getRadians() > constArm.SHOULDER_FORWARD_LIMIT && output > 0) {
      output = 0;
    }

    if (getShoulderPosition().getRadians() < constArm.SHOULDER_REVERSE_LIMIT && output < 0) {
      output = 0;
    }

    shoulderJoint.set(ControlMode.PercentOutput, output);
  }

  /**
   * Set the percent output of the elbow joint motor.
   * 
   * @param percent Percent output to set
   */
  public void setElbowPercentOutput(double percent) {

    double output = percent;

    if (getElbowPosition().getRadians() > constArm.ELBOW_FORWARD_LIMIT && output > 0) {
      output = 0;
    }

    if (getElbowPosition().getRadians() < constArm.ELBOW_REVERSE_LIMIT && output < 0) {
      output = 0;
    }

    elbowJoint.set(ControlMode.PercentOutput, output);
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
  public Rotation2d getShoulderPosition() {
    double rotations = shoulderEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.SHOULDER_ABSOLUTE_ENCODER_OFFSET);
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
  public Rotation2d getElbowPosition() {
    double rotations = elbowEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constArm.ELBOW_ABSOLUTE_ENCODER_OFFSET);
    rotations = MathUtil.inputModulus(rotations, -0.5, 0.5);

    if (constArm.ELBOW_ABSOLUTE_ENCODER_INVERT) {
      return Rotation2d.fromRotations(-rotations);
    } else {
      return Rotation2d.fromRotations(rotations);
    }

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

  public void resetPID() {
    shoulderPID.reset(getShoulderPosition().getRadians());
    elbowPID.reset(getElbowPosition().getRadians());
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("desiredGamePiece", desiredGamePiece.toString());
    SmartDashboard.putString("scoringLevel", scoringLevel.toString());
    SmartDashboard.putString("scoringColumn", scoringColumn.toString());

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder Raw", shoulderEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder Raw Inverted", 1 - shoulderEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Shoulder Motor Encoder Raw", shoulderJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Shoulder Position", getShoulderPosition().getDegrees());
      SmartDashboard.putNumber("Arm Shoulder Motor Output", shoulderJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw", elbowEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw Inverted", 1 - elbowEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Raw", elbowJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Elbow Position", getElbowPosition().getDegrees());
      SmartDashboard.putNumber("Arm Elbow Motor Output", elbowJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Tip Position X", Units.metersToInches(getArmTipPosition().getX()));
      SmartDashboard.putNumber("Arm Tip Position Y", Units.metersToInches(getArmTipPosition().getY()));
      SmartDashboard.putNumber("Arm Tip Distance",
          Units.metersToInches(getArmTipPosition().getDistance(new Translation2d())));

      SmartDashboard.putNumber("Arm Goal Angle Shoulder", goalShoulderAngle.getDegrees());
      SmartDashboard.putNumber("Arm Goal Angle Elbow", goalElbowAngle.getDegrees());

      SmartDashboard.putNumber("Arm PID Shoulder Goal", Units.radiansToDegrees(shoulderPID.getGoal().position));
      SmartDashboard.putNumber("Arm PID Shoudler Error", Units.radiansToDegrees(shoulderPID.getPositionError()));

      SmartDashboard.putNumber("Arm PID Elbow Goal", Units.radiansToDegrees(elbowPID.getGoal().position));
      SmartDashboard.putNumber("Arm PID Elbow Error", Units.radiansToDegrees(elbowPID.getPositionError()));
    }
  }
}
