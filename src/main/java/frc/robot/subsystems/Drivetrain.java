// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SN_SwerveModule;

public class Drivetrain extends SubsystemBase {

  private SN_SwerveModule[] modules;

  private AHRS navX;

  private SwerveDrivePoseEstimator poseEstimator;

  public Drivetrain() {

    modules = new SN_SwerveModule[] {
        new SN_SwerveModule(Constants.MODULE_0),
        new SN_SwerveModule(Constants.MODULE_1),
        new SN_SwerveModule(Constants.MODULE_2),
        new SN_SwerveModule(Constants.MODULE_3)
    };

    navX = new AHRS();
    navX.reset();
  }

  @Override
  public void periodic() {
  }
}
