// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  ArmState goalState;
  Rotation2d goalShoulderAngle;
  Rotation2d goalElbowAngle;

  int desiredNode;
  int gridChoice;
  ArmState armStateFromDesiredNode;

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

    goalState = ArmState.NONE;
    armStateFromDesiredNode = ArmState.NONE;

    desiredNode = 0;

    configure();
  }

  public void configure() {

    // shoulder config
    shoulderJoint.configFactoryDefault();

    shoulderConfig.slot0.kP = prefArm.shoulderP.getValue();
    shoulderConfig.slot0.kI = prefArm.shoulderI.getValue();
    shoulderConfig.slot0.kD = prefArm.shoulderD.getValue();

    shoulderConfig.motionCruiseVelocity = prefArm.shoulderMaxSpeed.getValue();
    shoulderConfig.motionAcceleration = prefArm.shoulderMaxAccel.getValue();

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

    elbowConfig.motionCruiseVelocity = prefArm.elbowMaxSpeed.getValue();
    elbowConfig.motionAcceleration = prefArm.elbowMaxAccel.getValue();

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
  public void setJointPositions(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
    double shoulderCounts = SN_Math.degreesToFalcon(shoulderAngle.getDegrees(), constArm.SHOULDER_GEAR_RATIO);
    shoulderJoint.set(ControlMode.MotionMagic, shoulderCounts);

    double elbowCounts = SN_Math.degreesToFalcon(elbowAngle.getDegrees(), constArm.ELBOW_GEAR_RATIO);
    elbowJoint.set(ControlMode.MotionMagic, elbowCounts);
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
   * Check if the arm is currently at the given state.
   * 
   * @return True if the arm is currently at the given state
   */
  public boolean isCurrentState(ArmState state) {
    for (int i = 0; i < ArmState.values().length - 1; i++) {
      return areJointsInToleranceToState(state);
    }
    return false;
  }

  /**
   * Check if a given joint rotation is within a given tolerance to a given goal
   * rotation.
   * 
   * @param jointRotation Current rotation of joint
   * @param goalRotation  Goal rotation of joint
   * @param tolerance     Tolerance of joint rotation
   * @return If the joint within tolerance of the goal
   */
  private boolean isJointInToleranceToAngle(Rotation2d jointRotation, Rotation2d goalRotation, Rotation2d tolerance) {
    double jointToGoal = Math.abs(jointRotation.getRadians() - goalRotation.getRadians());
    double fudgedTolerance = tolerance.getRadians() * prefArm.armToleranceFudgeFactor.getValue();

    return jointToGoal < fudgedTolerance;
  }

  /**
   * Check if the shoulder and elbow joints are within tolerance of a given state.
   * 
   * @param state State to check if joints are in tolerance to
   * @return If joints are in tolerance of a given state
   */
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
   * Get the goal arm state.
   * 
   * @return Goal arm state
   */
  public ArmState getGoalState() {
    return goalState;
  }

  /**
   * Set the goal arm state.
   * 
   * @param goalState Goal arm state
   */
  public void setGoalState(ArmState goalState) {
    this.goalState = goalState;
  }

  public boolean isGoalState(ArmState state) {
    return goalState == state;
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

  public boolean isCubeNode() {
    int gridlessNode = desiredNode % 9;
    return gridlessNode == 2 ||
        gridlessNode == 5;
  }

  public boolean isConeNode() {
    int gridlessNode = desiredNode % 9;
    return gridlessNode == 1 ||
        gridlessNode == 3 ||
        gridlessNode == 4 ||
        gridlessNode == 6;
  }

  public boolean isHighNode() {
    int gridlessNode = desiredNode % 9;
    return gridlessNode == 1 ||
        gridlessNode == 2 ||
        gridlessNode == 3;
  }

  public boolean isMidNode() {
    int gridlessNode = desiredNode % 9;
    return gridlessNode == 4 ||
        gridlessNode == 5 ||
        gridlessNode == 6;
  }

  public boolean isHybridNode() {
    int gridlessNode = desiredNode % 9;
    return gridlessNode == 7 ||
        gridlessNode == 8 ||
        gridlessNode == 9 ||
        gridlessNode == 0;
  }

  public boolean isValidNode() {
    return desiredNode > 0 && desiredNode <= 27;
  }

  /**
   * Set the desired node. 0 represents no desired node, and there are a total of
   * 27 nodes.
   * 
   * <pre>
   *1, 2, 3, 10, 11, 12, 19, 20, 21
   *4, 5, 6, 13, 14, 15, 22, 23, 24
   *7, 8, 9, 16, 17, 18, 25, 26, 27
   * </pre>
   * 
   * @param desiredNode Node to desire
   */
  public void setDesiredNode(int desiredNode) {
    this.desiredNode = MathUtil.clamp(desiredNode, 0, 27);
  }

  public int getDesiredGrid() {
    if (desiredNode <= 9) {
      return 1;
    } else if (desiredNode <= 18) {
      return 2;
    } else {
      return 3;
    }
  }

  public int getDesiredColumn() {
    if (!isValidNode()) {
      return 0;
    }

    if (isHybridNode()) {
      return desiredNode - (6 * getDesiredGrid());
    } else if (isMidNode()) {
      return (desiredNode + 3) - (6 * getDesiredGrid());
    } else if (isHighNode()) {
      return (desiredNode + 6) - (6 * getDesiredGrid());
    }

    return 0;
  }

  public void setGridChoice(int gridChoice) {
    this.gridChoice = MathUtil.clamp(gridChoice, 0, 3);
  }

  public void setArmStateFromDesiredNode() {
    switch (desiredNode % 9) {
      case 0:
        armStateFromDesiredNode = ArmState.HYBRID_SCORE;
        break;
      case 1:
        armStateFromDesiredNode = ArmState.HIGH_CONE_SCORE;
        break;
      case 2:
        armStateFromDesiredNode = ArmState.HIGH_CUBE_SCORE_PLACE;
        break;
      case 3:
        armStateFromDesiredNode = ArmState.HIGH_CONE_SCORE;
        break;
      case 4:
        armStateFromDesiredNode = ArmState.MID_CONE_SCORE;
        break;
      case 5:
        armStateFromDesiredNode = ArmState.MID_CUBE_SCORE;
        break;
      case 6:
        armStateFromDesiredNode = ArmState.MID_CONE_SCORE;
        break;
      case 7:
        armStateFromDesiredNode = ArmState.HYBRID_SCORE;
        break;
      case 8:
        armStateFromDesiredNode = ArmState.HYBRID_SCORE;
        break;
      default:
        armStateFromDesiredNode = ArmState.NONE;
        break;
    }

    if (desiredNode == 0) {
      armStateFromDesiredNode = ArmState.NONE;
    }

  }

  public Command prepPlaceCommand() {
    return prepStateFromStowCommand();
  }

  public Command stowCommand() {

    BooleanSupplier didArmScore = () -> (isGoalState(ArmState.HIGH_CONE_SCORE_LOWERED)
        || isGoalState(ArmState.MID_CONE_SCORE_LOWERED)
        || isCurrentState(ArmState.HIGH_CONE_SCORE)
        || isCurrentState(ArmState.MID_CONE_SCORE)
        || isCurrentState(ArmState.MID_CUBE_SCORE)
        || isCurrentState(ArmState.HIGH_CUBE_SCORE_PLACE));

    BooleanSupplier wasShelf = () -> isGoalState(ArmState.SHELF_INTAKE);

    BooleanSupplier wasHighCone = () -> isGoalState(ArmState.HIGH_CONE_SCORE_LOWERED);

    BooleanSupplier wasHighCube = () -> isGoalState(ArmState.HIGH_CUBE_SCORE_PLACE);
    BooleanSupplier wasMidCube = () -> isGoalState(ArmState.MID_CUBE_SCORE);

    return Commands.either(

        Commands.sequence(
            Commands.runOnce(() -> setGoalState(ArmState.HIGH_STOWED)),
            Commands.waitUntil(() -> isCurrentState(ArmState.HIGH_STOWED)),
            stateFromStowCommand(ArmState.LOW_STOWED)),

        Commands.either(

            Commands.either(

                Commands.sequence(
                    Commands.runOnce(() -> setGoalState(ArmState.CUBE_SCORE_LOW_TRANSITION)),
                    Commands.waitUntil(() -> isCurrentState(ArmState.CUBE_SCORE_LOW_TRANSITION)),
                    Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
                    Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
                    Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
                    .unless(() -> isGoalState(ArmState.LOW_STOWED)),

                Commands.either(

                    Commands.sequence(
                        Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
                        Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
                        Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
                        .unless(() -> isGoalState(ArmState.LOW_STOWED)),

                    Commands.either(Commands.sequence(
                        Commands.runOnce(() -> setGoalState(ArmState.HIGH_CONE_SCORE_TRANSITION)),
                        Commands.waitUntil(() -> isCurrentState(ArmState.HIGH_CONE_SCORE_TRANSITION)),
                        Commands.runOnce(() -> setGoalState(ArmState.CONE_SCORE_LOW_TRANSITION)),
                        Commands.waitUntil(() -> isCurrentState(ArmState.CONE_SCORE_LOW_TRANSITION)),
                        Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
                        Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
                        Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
                        .unless(() -> isGoalState(ArmState.LOW_STOWED)),
                        Commands.sequence(
                            Commands.runOnce(() -> setGoalState(ArmState.CONE_SCORE_LOW_TRANSITION)),
                            Commands.waitUntil(() -> isCurrentState(ArmState.CONE_SCORE_LOW_TRANSITION)),
                            Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
                            Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
                            Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
                            .unless(() -> isGoalState(ArmState.LOW_STOWED)),
                        wasHighCone),

                    wasMidCube),

                wasHighCube),

            Commands.sequence(
                Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
                Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
                Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
                .unless(() -> isGoalState(ArmState.LOW_STOWED)),

            didArmScore),
        wasShelf);
  }

  public Command intakeFloorDeployCommand() {
    // return Commands.sequence(
    // Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
    // Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
    // Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE_TRANSITION)),
    // Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE_TRANSITION)),
    // Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE)),
    // Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE)))
    // .unless(() -> isGoalState(ArmState.FLOOR_INTAKE));

    return Commands.sequence(
        Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE_TRANSITION)),
        Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE_TRANSITION)),
        Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE)),
        Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE)))
        .unless(() -> isGoalState(ArmState.FLOOR_INTAKE));

  }

  public Command intakeFloorStowCommand() {
    // return Commands.sequence(
    // Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE_TRANSITION)),
    // Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE_TRANSITION)),
    // Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
    // Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
    // Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
    // .unless(() -> !isGoalState(ArmState.FLOOR_INTAKE));

    return Commands.sequence(
        Commands.runOnce(() -> setGoalState(ArmState.FLOOR_INTAKE_TRANSITION)),
        Commands.waitUntil(() -> isCurrentState(ArmState.FLOOR_INTAKE_TRANSITION)),
        Commands.runOnce(() -> setGoalState(ArmState.LOW_STOWED)))
        .unless(() -> !isGoalState(ArmState.FLOOR_INTAKE));
  }

  public Command stateFromStowCommand(ArmState state) {
    return Commands.sequence(
        Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
        Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
        Commands.runOnce(() -> setGoalState(state)))
        .unless(() -> isGoalState(state));
  }

  public Command prepStateFromStowCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setGoalState(ArmState.MID_STOWED)),
        Commands.waitUntil(() -> isCurrentState(ArmState.MID_STOWED)),
        Commands.runOnce(() -> setGoalState(armStateFromDesiredNode)))
        .unless(() -> isGoalState(armStateFromDesiredNode));
  }

  public int getDesiredNode() {
    return desiredNode;
  }

  public int getGridChoice() {
    return gridChoice;
  }

  public boolean getGridOneValue() {
    return gridChoice == 1;
  }

  public boolean getGridTwoValue() {
    return gridChoice == 2;
  }

  public boolean getGridThreeValue() {
    return gridChoice == 3;
  }

  public boolean getNodeOneValue() {
    return desiredNode == 1;
  }

  public boolean getNodeTwoValue() {
    return desiredNode == 2;
  }

  public boolean getNodeThreeValue() {
    return desiredNode == 3;
  }

  public boolean getNodeFourValue() {
    return desiredNode == 4;
  }

  public boolean getNodeFiveValue() {
    return desiredNode == 5;
  }

  public boolean getNodeSixValue() {
    return desiredNode == 6;
  }

  public boolean getNodeSevenValue() {
    return desiredNode == 7;
  }

  public boolean getNodeEightValue() {
    return desiredNode == 8;
  }

  public boolean getNodeNineValue() {
    return desiredNode == 9;
  }

  public boolean getNodeTenValue() {
    return desiredNode == 10;
  }

  public boolean getNodeElevenValue() {
    return desiredNode == 11;
  }

  public boolean getNodeTwelveValue() {
    return desiredNode == 12;
  }

  public boolean getNodeThirteenValue() {
    return desiredNode == 13;
  }

  public boolean getNodeFourteenValue() {
    return desiredNode == 14;
  }

  public boolean getNodeFifteenValue() {
    return desiredNode == 15;
  }

  public boolean getNodeSixteenValue() {
    return desiredNode == 16;
  }

  public boolean getNodeSeventeenValue() {
    return desiredNode == 17;
  }

  public boolean getNodeEighteenValue() {
    return desiredNode == 18;
  }

  public boolean getNodeNineteenValue() {
    return desiredNode == 19;
  }

  public boolean getNodeTwentyValue() {
    return desiredNode == 20;
  }

  public boolean getNodeTwentyOneValue() {
    return desiredNode == 21;
  }

  public boolean getNodeTwentyTwoValue() {
    return desiredNode == 22;
  }

  public boolean getNodeTwentyThreeValue() {
    return desiredNode == 23;
  }

  public boolean getNodeTwentyFourValue() {
    return desiredNode == 24;
  }

  public boolean getNodeTwentyFiveValue() {
    return desiredNode == 25;
  }

  public boolean getNodeTwentySixValue() {
    return desiredNode == 26;
  }

  public boolean getNodeTwentySevenValue() {
    return desiredNode == 27;
  }

  @Override
  public void periodic() {

    setArmStateFromDesiredNode();

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Arm Shoulder Absolute Encoder Raw", shoulderEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Shoulder Motor Encoder Raw", shoulderJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Shoulder Motor Encoder Velocity Raw", shoulderJoint.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Arm Shoulder Position", getShoulderPosition().getDegrees());
      SmartDashboard.putNumber("Arm Shoulder Motor Output", shoulderJoint.getMotorOutputPercent());

      SmartDashboard.putNumber("Arm Elbow Absolute Encoder Raw", elbowEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Raw", elbowJoint.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Elbow Motor Encoder Velocity Raw", elbowJoint.getSelectedSensorVelocity());
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

      for (ArmState state : ArmState.values()) {
        SmartDashboard.putBoolean("Arm at state " + state.toString(), isCurrentState(state));
      }
      SmartDashboard.putString("Arm Goal State", getGoalState().toString());

      SmartDashboard.putNumber("Arm Desired Node", desiredNode);
      SmartDashboard.putBoolean("Arm Is High Node", isHighNode());
      SmartDashboard.putBoolean("Arm Is Mid Node", isMidNode());
      SmartDashboard.putBoolean("Arm Is Hybrid Node", isHybridNode());
      SmartDashboard.putBoolean("Arm Is Cube Node", isCubeNode());
      SmartDashboard.putBoolean("Arm Is Cone Node", isConeNode());
    }
  }
}
