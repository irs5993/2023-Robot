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
        new PIDController(0.065, 0, 0.01),
        // This should return the measurement
        drivetrainSubsystem::getPitch,
        // This should return the setpoint (can also be a constant)
        -1,
        // This uses the output
        output -> {
          drivetrainSubsystem.driveNormal(-MathUtil.clamp(output, -0.4, 0.4), 0);
        });

    addRequirements(drivetrainSubsystem);

    getController().setTolerance(5);
    SmartDashboard.putBoolean("Is Balanced", getController().atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
