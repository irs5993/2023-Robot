// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveAngleCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private double angle;

  private final PIDController pid = new PIDController(0.06, 0, 0.018);

  public DriveAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.angle = angle;
    pid.setTolerance(1);
  }

  @Override
  public void execute() {
    double rotation = pid.calculate(drivetrainSubsystem.getYaw(), angle);
    if (pid.atSetpoint()) {
      drivetrainSubsystem.drive(Constants.MotorSpeedValues.MEDIUM, 0);
    } else {
      drivetrainSubsystem.drive(0, rotation);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
