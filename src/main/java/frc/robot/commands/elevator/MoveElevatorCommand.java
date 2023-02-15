// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final DoubleSupplier frontSpeedSupplier, rearSpeedSupplier;

  public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier frontSpeedSupplier, DoubleSupplier rearSpeedSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    this.frontSpeedSupplier = frontSpeedSupplier;
    this.rearSpeedSupplier = rearSpeedSupplier;
  }

  @Override
  public void execute() {
    elevatorSubsystem.setFrontElevatorSpeed(frontSpeedSupplier.getAsDouble());
    elevatorSubsystem.setRearElevatorSpeed(rearSpeedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
