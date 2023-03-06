// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator.presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class OrientTargetCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double rear_elevator_speed = Constants.MotorSpeedValues.MEDIUM, front_elevator_speed = Constants.MotorSpeedValues.MEDIUM;
  private final PIDController pid = new PIDController(0.1, 0, 0.02);
  private final double SETPOINT = -80;
  
  public OrientTargetCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 
  }

  public OrientTargetCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    rear_elevator_speed = Math.abs(speed);
    front_elevator_speed = Math.abs(speed);
  }

  @Override
  public void execute() {
    // elevatorSubsystem.setFrontElevatorSpeed(front_elevator_speed);

    //double rear = pid.calculate(elevatorSubsystem.getEncoderRaw(), SETPOINT);
    if (elevatorSubsystem.getEncoderRaw() >= -4500) {
      elevatorSubsystem.setRearElevatorSpeed(0.7);
    } else {
      elevatorSubsystem.setRearElevatorSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
