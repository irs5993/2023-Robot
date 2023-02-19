// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class RunGripperCommand extends CommandBase {
  private final GripperSubsystem gripperSubsystem;
  private double speed;

  public RunGripperCommand(GripperSubsystem gripperSubsystem, double speed) {
    this.gripperSubsystem = gripperSubsystem;
    addRequirements(gripperSubsystem);

    this.speed = speed;
  }

  @Override
  public void execute() {
    this.gripperSubsystem.setMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    this.gripperSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
