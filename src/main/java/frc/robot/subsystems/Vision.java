// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  RobotPoseEstimator photonPoseEstimator;

  public Vision() {
    // TODO: Change to current april tag array upon new WPI update
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(null, 0, 0);

    // add more cameras here & to the cameraList when possible
    PhotonCamera lifecam = new PhotonCamera(constVision.lifecamPhotonName);
    Transform3d robotToLifecam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    ArrayList<Pair<PhotonCamera, Transform3d>> cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    cameraList.add(new Pair<PhotonCamera, Transform3d>(lifecam, robotToLifecam));
    photonPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        cameraList);
  }

  // Returns the current estimated pose from vision & it's timestamp.
  public Pair<Pose2d, Double> getPoseFromVision(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = photonPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }
  }

  @Override
  public void periodic() {
  }
}
