// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private double speed = Constants.MotorSpeedValues.MEDIUM;

  public ExtendArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  public ExtendArmCommand(ArmSubsystem armSubsystem, double speed) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    this.speed = Math.abs(speed);
  }

  @Override
  public void execute() {
    armSubsystem.setMotorSpeed(-speed);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.getTopSwitch();
  }
}
