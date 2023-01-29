// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.utils.SN_Math;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_CANSparkMax pivotMotor;
  SN_CANSparkMax rollerMotor;

  AbsoluteEncoder absolutePivotEncoder;

  TalonFXConfiguration pivotMotorConfig;

  public Collector() {
    pivotMotor = new SN_CANSparkMax(mapCollector.PIVOT_MOTOR_CAN);
    rollerMotor = new SN_CANSparkMax(mapCollector.ROLLER_MOTOR_CAN);

    absolutePivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    pivotMotorConfig = new TalonFXConfiguration();
    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    rollerMotor.configFactoryDefault();

    pivotMotorConfig.slot0.kP = prefCollector.collectorP.getValue();
    pivotMotorConfig.slot0.kI = prefCollector.collectorI.getValue();
    pivotMotorConfig.slot0.kD = prefCollector.collectorD.getValue();

    pivotMotorConfig.forwardSoftLimitEnable = prefCollector.collectorForwardSoftLimitEnable.getValue();
    pivotMotorConfig.reverseSoftLimitEnable = prefCollector.collectorReverseSoftLimitEnable.getValue();

    pivotMotor.encoder.setPositionConversionFactor(SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    pivotMotorConfig.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.FORWARD_LIMIT),
        constCollector.GEAR_RATIO);
    pivotMotorConfig.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.REVERSE_LIMIT),
        constCollector.GEAR_RATIO);

    pivotMotorConfig.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefCollector.collectorAllowableClosedLoopErrorDegrees.getValue(),
            constCollector.GEAR_RATIO);
    pivotMotorConfig.slot0.closedLoopPeakOutput = prefCollector.collectorClosedLoopPeakOutput.getValue();

    pivotMotor.configAllSettings(pivotMotorConfig);
    resetCollectorToAbsolute();
  }

  /**
   * Reset the pivot motor encoder counts to 0
   */
  public void resetPivotMotorEncoder() {
    pivotMotor.setSelectedSensorPosition(0);
  }

  public void spinRollerMotor(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  // TODO: Set PID values
  public void setPivotMotorAngle(double angle) {
    angle = SN_Math.degreesToFalcon(angle, constCollector.GEAR_RATIO);
    setPivotMotorPosition(angle);
  }

  public void setPivotMotorPosition(double position) {
    pivotMotor.set(ControlMode.Position, position);
  }

  public void setPivotMotorSpeed(double speed) {
    pivotMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getCollectorAbsoluteEncoder() {
    return absolutePivotEncoder.getPosition();
  }

  private void resetCollectorToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(getCollectorAbsoluteEncoder(),
        constCollector.GEAR_RATIO);
    pivotMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Collector Pivot Motor Encoder", pivotMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder", getCollectorAbsoluteEncoder());
    }
  }
}
