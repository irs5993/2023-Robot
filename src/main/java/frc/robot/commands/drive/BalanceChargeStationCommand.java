// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceChargeStationCommand extends PIDCommand {
  public BalanceChargeStationCommand(DrivetrainSubsystem drivetrainSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.065, 0, 0.015),
        // This should return the measurement
        drivetrainSubsystem::getPitch,
        // This should return the setpoint (can also be a constant)
        0.5,
        // This uses the output
        output -> {
          drivetrainSubsystem.drive(MathUtil.clamp(output, -0.6, 0.6), 0);
        });

    addRequirements(drivetrainSubsystem);

    getController().setTolerance(1);
    SmartDashboard.putBoolean("is on setpoint", getController().atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
