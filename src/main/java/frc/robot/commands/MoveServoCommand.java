// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class MoveServoCommand extends CommandBase {
  private final VisionSubsystem visionSubsystem;
  private final DoubleSupplier angleSupplier;

  private final double MAX_ANGLE = 160;
  private final double MIN_ANGLE = 100;

  public MoveServoCommand(VisionSubsystem visionSubsystem, DoubleSupplier angleSupplier) {
    this.visionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem); 

    this.angleSupplier = angleSupplier;
  }

  @Override
  public void execute() {
    double value = angleSupplier.getAsDouble();
    double calculated = (value + 1) * (MAX_ANGLE - MIN_ANGLE) / 2 + MIN_ANGLE;
    visionSubsystem.setServoAngle(calculated);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
