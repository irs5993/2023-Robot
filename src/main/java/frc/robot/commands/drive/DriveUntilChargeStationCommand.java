// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveUntilChargeStationCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private double xSpeed;

  public DriveUntilChargeStationCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
    this.xSpeed = xSpeed;
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(xSpeed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return drivetrainSubsystem.getPitch() < -10;
  }
}
