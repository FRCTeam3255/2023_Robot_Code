// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {

  TalonFX pivotMotor;
  CANSparkMax rollerMotor;

  DutyCycleEncoder absoluteEncoder;

  TalonFXConfiguration pivotConfig;

  /** Creates a new Collector. */
  public Collector() {

    pivotMotor = new TalonFX(mapCollector.PIVOT_MOTOR_CAN);
    rollerMotor = new CANSparkMax(mapCollector.ROLLER_MOTOR_CAN, MotorType.kBrushless);

    absoluteEncoder = new DutyCycleEncoder(mapCollector.PIVOT_ABSOLUTE_ENCODER_DIO);

    pivotConfig = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();

    pivotConfig.slot0.kP = prefCollector.pivotP.getValue();
    pivotConfig.slot0.kI = prefCollector.pivotI.getValue();
    pivotConfig.slot0.kD = prefCollector.pivotD.getValue();

    pivotConfig.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(
        prefCollector.pivotTolerance.getValue(),
        constCollector.GEAR_RATIO);

    pivotConfig.motionCruiseVelocity = prefCollector.pivotMaxSpeed.getValue();
    pivotConfig.motionAcceleration = prefCollector.pivotMaxAccel.getValue();

    pivotMotor.configAllSettings(pivotConfig);

    pivotMotor.setInverted(constCollector.PIVOT_MOTOR_INVERT);

    pivotMotor.configForwardSoftLimitEnable(true);
    pivotMotor.configReverseSoftLimitEnable(true);
    pivotMotor.configForwardSoftLimitThreshold(constCollector.PIVOT_FORWARD_LIMIT_VALUE);
    pivotMotor.configReverseSoftLimitThreshold(constCollector.PIVOT_REVERSE_LIMIT_VALUE);

    pivotMotor.setNeutralMode(constCollector.PIVOT_MOTOR_NEUTRAL_MODE);

    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setInverted(constCollector.ROLLER_MOTOR_INVERT);
    rollerMotor.setIdleMode(constCollector.ROLLER_MOTOR_NEUTRAL_MODE);
  }

  public void setPivotAngle(Rotation2d angle) {
    pivotMotor.set(ControlMode.MotionMagic, SN_Math.degreesToFalcon(angle.getDegrees(), constCollector.GEAR_RATIO));
  }

  public void setPivotAngle(SN_DoublePreference degrees) {
    setPivotAngle(Rotation2d.fromDegrees(degrees.getValue()));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  private Rotation2d getAbsoluteEncoder() {
    double rotations = absoluteEncoder.get() / 2;
    rotations -= Units.radiansToRotations(constCollector.ABSOLUTE_ENCODER_OFFSET);
    rotations = MathUtil.inputModulus(rotations, -0.5, 0.5);

    if (constCollector.ABSOLUTE_ENCODER_INVERT) {
      return Rotation2d.fromRotations(-rotations);
    } else {
      return Rotation2d.fromRotations(rotations);
    }
  }

  public void resetPivotMotorToAbsolute() {
    double counts = SN_Math.degreesToFalcon(getAbsoluteEncoder().getDegrees(), constCollector.GEAR_RATIO);
    pivotMotor.setSelectedSensorPosition(counts);
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(pivotMotor.getSelectedSensorPosition(), constCollector.GEAR_RATIO));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (Constants.OUTPUT_DEBUG_VALUES) {

      SmartDashboard.putNumber("Collector Pivot Angle", getPivotAngle().getDegrees());
      SmartDashboard.putNumber("Collector Pivot Motor Encoder Counts Position", pivotMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Collector Pivot Motor Encoder Counts Velocity", pivotMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Collector Pivot Motor Output Percent", pivotMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Collector Absolute Encoder Raw / 2", absoluteEncoder.get() / 2);
      SmartDashboard.putNumber("Collector Absolute Encoder", getAbsoluteEncoder().getDegrees());

    }
  }
}
