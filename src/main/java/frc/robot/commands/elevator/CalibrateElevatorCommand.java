// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class CalibrateElevatorCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  public CalibrateElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setRearElevatorSpeed(-Constants.MotorSpeedValues.HIGH);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.resetEncoder();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getRearBottomSwitch();
  }
}
