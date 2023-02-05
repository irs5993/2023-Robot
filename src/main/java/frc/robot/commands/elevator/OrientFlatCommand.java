// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeed;

public class OrientFlatCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double rear_elevator_voltage = Constants.MotorSpeedValues.MEDIUM, front_elevator_voltage = Constants.MotorSpeedValues.MEDIUM;
  
  public OrientFlatCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 
  }

  public OrientFlatCommand(ElevatorSubsystem elevatorSubsystem, MotorSpeed speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    switch(speed) {
      case LOW:
        rear_elevator_voltage = Constants.MotorSpeedValues.LOW;
        front_elevator_voltage = Constants.MotorSpeedValues.LOW;
        break;
      case MEDIUM:
        break;
      case HIGH:
        rear_elevator_voltage = Constants.MotorSpeedValues.HIGH;
        front_elevator_voltage = Constants.MotorSpeedValues.HIGH;
        break;
      case MAX:
        rear_elevator_voltage = Constants.MotorSpeedValues.MAX;
        front_elevator_voltage = Constants.MotorSpeedValues.MAX;
        break;
    }
  }

  public OrientFlatCommand(ElevatorSubsystem elevatorSubsystem, double voltage) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    rear_elevator_voltage = Math.abs(voltage);
    front_elevator_voltage = Math.abs(voltage);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setRearElevatorVoltage(-rear_elevator_voltage);
    elevatorSubsystem.setFrontElevatorVoltage(-front_elevator_voltage);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getRearBottomSwitch() && elevatorSubsystem.getFrontBottomSwitch();
  }
}
