// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DynamicDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final DoubleSupplier xSpeedSupplier, zRotationSupplier;

  public DynamicDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier zRotationSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem); 

    this.xSpeedSupplier = xSpeedSupplier;
    this.zRotationSupplier = zRotationSupplier;
  }


  @Override
  public void execute() {
    drivetrainSubsystem.drive(xSpeedSupplier.getAsDouble(), zRotationSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
