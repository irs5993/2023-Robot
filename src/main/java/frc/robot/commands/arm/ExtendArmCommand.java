// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Timer timer;
  private double speed = Constants.MotorSpeedValues.MEDIUM;

  public ExtendArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    timer = new Timer();
  }

  public ExtendArmCommand(ArmSubsystem armSubsystem, double speed) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    timer = new Timer();
    this.speed = Math.abs(speed);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
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
    return timer.hasElapsed(3);
  }
}
