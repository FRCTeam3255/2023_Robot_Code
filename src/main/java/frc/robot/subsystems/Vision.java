// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  RobotPoseEstimator photonPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;

  public Vision() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout! Must be on WPILIB 2023.2.1+" + e);
    }

    // TODO: MOVE TRANSFORMS TO CONSTANTS
    PhotonCamera ARCamera = new PhotonCamera(constVision.ARPhotonName);
    Transform3d robotToAR = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    PhotonCamera OVCamera = new PhotonCamera(constVision.OVPhotonName);
    Transform3d robotToOV = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    PhotonCamera lifecam = new PhotonCamera(constVision.lifecamPhotonName);
    Transform3d robotToLifecam = new Transform3d(new Translation3d(0.4191, 0.1905, 0.6604), new Rotation3d(0, 0, 0));

    ArrayList<Pair<PhotonCamera, Transform3d>> cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    // cameraList.add(new Pair<PhotonCamera, Transform3d>(ARCamera, robotToAR));
    // cameraList.add(new Pair<PhotonCamera, Transform3d>(OVCamera, robotToOV));
    cameraList.add(new Pair<PhotonCamera, Transform3d>(lifecam, robotToLifecam));

    photonPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        cameraList);
  }

  // Returns the current estimated pose from vision & it's timestamp.
  public Optional<Pair<Pose3d, Double>> getPoseFromVision(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    return photonPoseEstimator.update();
  }

  @Override
  public void periodic() {
  }
}
