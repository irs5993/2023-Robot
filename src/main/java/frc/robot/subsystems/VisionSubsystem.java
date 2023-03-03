// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  
  public VisionSubsystem() {
    camera = new PhotonCamera("main");
  }

  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public PhotonTrackedTarget getBestTarget() {
    var result = getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }
}
