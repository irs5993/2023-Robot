// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FollowTargetCommand extends CommandBase {
  private final VisionSubsystem visionSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final PIDController pid = new PIDController(0.06, 0, 0.018);

  public FollowTargetCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.visionSubsystem = visionSubsystem;

    pid.setTolerance(2);

    addRequirements(visionSubsystem, drivetrainSubsystem);
  }

  @Override
  public void execute() {
    var target = visionSubsystem.getBestTarget();
    if (target != null) {
      double rotation = -pid.calculate(target.getYaw());
      drivetrainSubsystem.drive(0.56, MathUtil.clamp(rotation, -0.6, 0.6));
    } else {
      drivetrainSubsystem.drive(0, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
