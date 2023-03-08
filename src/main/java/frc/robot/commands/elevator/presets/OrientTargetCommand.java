// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator.presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class OrientTargetCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double rear_elevator_speed = Constants.MotorSpeedValues.MAX, front_elevator_speed = Constants.MotorSpeedValues.LOW;

  private final double SETPOINT = 6200;
  private final double TOLERANCE = 50;
  private boolean atSetpoint = false;

  private boolean upfirst = false;
  
  public OrientTargetCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem); 

    if (elevatorSubsystem.getEncoderRaw() > 4150) {
      upfirst = true;
    }
  }

  @Override
  public void execute() {
    if (upfirst) {
      elevatorSubsystem.setFrontElevatorSpeed(front_elevator_speed);

      if (elevatorSubsystem.getFrontTopSwitch()) {
        levelRear();
      }
    } else {
      levelRear();
      if (atSetpoint) {
        elevatorSubsystem.setFrontElevatorSpeed(front_elevator_speed);
      }
    }
  }

  private void levelRear() {
    double encoder = elevatorSubsystem.getEncoderRaw();
    if (encoder < SETPOINT - TOLERANCE) {
      elevatorSubsystem.setRearElevatorSpeed(rear_elevator_speed);
    } else if (encoder > SETPOINT + TOLERANCE) {
      elevatorSubsystem.setRearElevatorSpeed(-rear_elevator_speed);
    } else {
      atSetpoint = true;
      elevatorSubsystem.setRearElevatorSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    atSetpoint = false;
  }

  @Override
  public boolean isFinished() {
    return atSetpoint && elevatorSubsystem.getFrontTopSwitch();
  }
}
