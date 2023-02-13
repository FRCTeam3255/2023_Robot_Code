// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  PhotonPoseEstimator ARCameraPoseEstimator;
  PhotonPoseEstimator OVCameraPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;

  public Vision() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath().resolve("ApriltagLocations.json"));
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout! Must be on WPILIB 2023.2.1+" + e);
    }

    PhotonCamera ARCamera = new PhotonCamera(constVision.ARPhotonName);
    Transform3d robotToAR = constVision.robotToAR;

    PhotonCamera OVCamera = new PhotonCamera(constVision.OVPhotonName);
    Transform3d robotToOV = constVision.robotToOV;

    PhotonCamera lifecam = new PhotonCamera(constVision.lifecamPhotonName);
    Transform3d robotToLifecam = constVision.robotToLifecam;

    // Create an instance of this for every camera you want to do pose estimation
    // with, as well as a getPoseFrom__ method to reference in the
    // AddVisionMeasurement command
    ARCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        ARCamera,
        robotToAR);

    OVCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        OVCamera,
        robotToOV);
  }

  public Optional<EstimatedRobotPose> getPoseFromARCamera(Pose2d referencePose) {
    ARCameraPoseEstimator.setReferencePose(referencePose);
    return ARCameraPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getPoseFromOVCamera(Pose2d referencePose) {
    OVCameraPoseEstimator.setReferencePose(referencePose);
    return OVCameraPoseEstimator.update();
  }

  @Override
  public void periodic() {
  }
}
