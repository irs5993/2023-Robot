// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeed;

public class OrientUpwardCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double rear_elevator_speed = Constants.MotorSpeedValues.MEDIUM, front_elevator_speed = Constants.MotorSpeedValues.MEDIUM;
  
  public OrientUpwardCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 
  }

  public OrientUpwardCommand(ElevatorSubsystem elevatorSubsystem, MotorSpeed speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    switch(speed) {
      case LOW:
        rear_elevator_speed = Constants.MotorSpeedValues.LOW;
        front_elevator_speed = Constants.MotorSpeedValues.LOW;
        break;
      case MEDIUM:
        break;
      case HIGH:
        rear_elevator_speed = Constants.MotorSpeedValues.HIGH;
        front_elevator_speed = Constants.MotorSpeedValues.HIGH;
        break;
      case MAX:
        rear_elevator_speed = Constants.MotorSpeedValues.MAX;
        front_elevator_speed = Constants.MotorSpeedValues.MAX;
        break;
    }
  }

  public OrientUpwardCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    rear_elevator_speed = Math.abs(speed);
    front_elevator_speed = Math.abs(speed);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setRearElevatorSpeed(-rear_elevator_speed);
    elevatorSubsystem.setFrontElevatorSpeed(front_elevator_speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getRearBottomSwitch() && elevatorSubsystem.getFrontTopSwitch();
  }
}