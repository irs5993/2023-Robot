// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetRearElevatorPositionCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final int position;
    
  public SetRearElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, int position) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 
    
    this.position = position;
  }

  @Override
  public void execute() {
    elevatorSubsystem.setRearElevatorPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    elevatorSubsystem.resetRearSetpoint();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.rearAtSetpoint();
  }
}
