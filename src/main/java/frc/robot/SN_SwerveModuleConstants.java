package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class SN_SwerveModuleConstants {
    public final int driveMotorID;
    public final int steerMotorID;
    public final int steerEncoderID;
    public final double steerEncoderOffset;
    public final Translation2d modulePosition;
    public final int moduleNumber;

    /**
     * Swerve Module Constants to use when creating swerve modules. Units are not
     * defined here and are left up to the implementation
     * 
     * @param driveMotorID
     * @param steerMotorID
     * @param steerEncoderID
     * @param steerEncodersteerEncoderOffsetOffset
     * @param modulePosition
     * @param moduleNumber
     */
    public SN_SwerveModuleConstants(
            int driveMotorID,
            int steerMotorID,
            int steerEncoderID,
            double steerEncoderOffset,
            Translation2d modulePosition,
            int moduleNumber) {
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.steerEncoderID = steerEncoderID;
        this.steerEncoderOffset = steerEncoderOffset;
        this.modulePosition = modulePosition;
        this.moduleNumber = moduleNumber;
    }
}
