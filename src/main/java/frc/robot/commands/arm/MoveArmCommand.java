// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private double speed;

  public MoveArmCommand(ArmSubsystem armSubsystem, double speed) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    this.speed = speed;
  }

  @Override
  public void execute() {
    armSubsystem.setMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
