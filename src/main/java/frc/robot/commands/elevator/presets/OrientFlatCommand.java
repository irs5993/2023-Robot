// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator.presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class OrientFlatCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double front_elevator_speed = 0.23;

  private final PIDController pid = new PIDController(1, 0, 0);
    
  public OrientFlatCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    pid.setTolerance(20);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setFrontElevatorSpeed(-front_elevator_speed);
    elevatorSubsystem.setRearElevatorSpeed(pid.calculate(elevatorSubsystem.getEncoderRaw(), 600));
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    elevatorSubsystem.resetRearSetpoint();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.rearAtSetpoint() && elevatorSubsystem.getFrontBottomSwitch();
  }
}
