// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator.presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class OrientUpwardCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double front_elevator_speed = 0.21;
    
  public OrientUpwardCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 
  }

  @Override
  public void execute() {
    elevatorSubsystem.setFrontElevatorSpeed(front_elevator_speed);
    elevatorSubsystem.setRearElevatorPosition(4150);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    elevatorSubsystem.resetRearSetpoint();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.rearAtSetpoint() && elevatorSubsystem.getFrontTopSwitch();
  }
}
