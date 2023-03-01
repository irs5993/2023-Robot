// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

public class DriveAngleCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final double MAX_OUT = 0.7;
  private final double TOLERANCE = 2;

  private double angle;

  private final PIDController pid = new PIDController(Constants.RobotConstants.kP, Constants.RobotConstants.kI, Constants.RobotConstants.kP);

  public DriveAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.angle = angle;
    pid.setTolerance(TOLERANCE);
    pid.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    double rotation = pid.calculate(drivetrainSubsystem.getYaw(), angle);
    if (pid.atSetpoint()) {
      drivetrainSubsystem.drive(Constants.MotorSpeedValues.MEDIUM, 0);
    } else {
      drivetrainSubsystem.drive(0, MathUtil.clamp(rotation, -MAX_OUT, MAX_OUT));
    }
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
