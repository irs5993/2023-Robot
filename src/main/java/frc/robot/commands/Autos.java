// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.drive.ConstantDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DrivetrainSubsystem drivetrainSubsystem) {
    return Commands.sequence(
      new ConstantDriveCommand(drivetrainSubsystem, 0.55, 0).withTimeout(2),
      new ConstantDriveCommand(drivetrainSubsystem, -0.55, 0).withTimeout(2),
      new ConstantDriveCommand(drivetrainSubsystem, 0, -0.55).withTimeout(2)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
