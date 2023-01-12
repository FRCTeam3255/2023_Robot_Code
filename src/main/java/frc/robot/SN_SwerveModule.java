// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.frcteam3255.components.motors.SN_TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotPreferences.prefDrivetrain;

/**
 * Coaxial swerve module with SN_TalonFX's for both driving and steering. Uses
 * CANCoder's for the absolute steering encoders.
 */
public class SN_SwerveModule {

    public int moduleNumber;

    private SN_TalonFX driveMotor;
    private SN_TalonFX steerMotor;

    private CANCoder absoluteEncoder;
    private double absoluteEncoderOffset;

    private TalonFXConfiguration driveConfiguration;
    private TalonFXConfiguration steerConfiguration;

    private double lastAngle;

    /**
     * Create a new SN_SwerveModule.
     * 
     * @param moduleConstants Constants required to create a swerve module
     */
    public SN_SwerveModule(SN_SwerveModuleConstants moduleConstants) {
        moduleNumber = moduleConstants.moduleNumber;

        driveMotor = new SN_TalonFX(moduleConstants.driveMotorID);
        steerMotor = new SN_TalonFX(moduleConstants.steerMotorID);

        absoluteEncoder = new CANCoder(moduleConstants.absoluteEncoderID);
        absoluteEncoderOffset = moduleConstants.absoluteEncoderOffset;

        driveConfiguration = new TalonFXConfiguration();
        steerConfiguration = new TalonFXConfiguration();

        lastAngle = 0;

        configure();
    }

    /**
     * Configure the drive motor, steer motor, and absolute encoder.
     */
    public void configure() {
        driveMotor.configFactoryDefault();
        // TODO: set driveConfiguration to preferences
        driveMotor.configAllSettings(driveConfiguration);
        // TODO: set drive motor configs from constants

        steerMotor.configFactoryDefault();
        // TODO: set steerConfiguration to preferences
        steerMotor.configAllSettings(steerConfiguration);
        // TODO: set steer motor configs from constants

        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.setPositionToAbsolute();
    }

    /**
     * Set the desired state of the swerve module. The state includes the velocity
     * and angles of the module. The state that the motors are actually set to will
     * not always be the desired state, rather it will optimized to reduce
     * unnecessary steering in favor of reversing the direction of the drive motor.
     * 
     * @param desiredState    Desired velocity and angle of the module
     * @param isDriveOpenLoop Is the drive motor velocity set using open or closed
     *                        loop control
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {

        // TODO: implement

    }

    /**
     * Neutral the drive motor output.
     */
    public void neutralDriveOutput() {
        driveMotor.neutralOutput();
    }

    /**
     * Reset the steer motor encoder to match the angle of the absolute encoder.
     */
    public void resetSteerMotorEncoderToAbsolute() {
        // TODO: implement
    }

    /**
     * Get the absolute encoder with an offset applied. This angle will match the
     * current angle of the module wheel.
     * 
     * @return Absolute encoder with offset applied
     */
    public Rotation2d getAbsoluteEncoder() {

        // TODO: implement

        return null;
    }

    /**
     * Reset the drive motor encoder count to 0.
     */
    public void resetDriveEncoderCount() {
        driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Get the current state of the swerve module. State includes a velocity and
     * angle.
     * 
     * @return State of swerve module
     */
    public SwerveModuleState getState() {

        // TODO: implement

        return null;
    }

}
