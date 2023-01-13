package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class SN_SwerveModuleConstants {
  public final int driveMotorID;
  public final int steerMotorID;
  public final int absoluteEncoderID;
  public final double absoluteEncoderOffset;
  public final Translation2d modulePosition;
  public final int moduleNumber;

  /**
   * Swerve Module Constants to use when creating swerve modules. Units are not
   * defined here and are left up to the implementation
   * 
   * @param driveMotorID
   * @param steerMotorID
   * @param absoluteEncoderID
   * @param absoluteEncoderOffset
   * @param modulePosition
   * @param moduleNumber
   */
  public SN_SwerveModuleConstants(
      int driveMotorID,
      int steerMotorID,
      int absoluteEncoderID,
      double absoluteEncoderOffset,
      Translation2d modulePosition,
      int moduleNumber) {
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.absoluteEncoderID = absoluteEncoderID;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.modulePosition = modulePosition;
    this.moduleNumber = moduleNumber;
  }
}
