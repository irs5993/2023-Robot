// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.drive.BalanceChargeStationCommand;
import frc.robot.commands.drive.ConstantDriveCommand;
import frc.robot.commands.drive.DriveUntilChargeStationCommand;
import frc.robot.commands.elevator.presets.OrientUpwardCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  public static CommandBase ScoreBalanceAuto(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
    return Commands.sequence(
      new OrientUpwardCommand(elevatorSubsystem),
      new ExtendArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new WaitCommand(1),
      new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX).withTimeout(2),
      new RetractArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new DriveUntilChargeStationCommand(drivetrainSubsystem, -Constants.MotorSpeedValues.MEDIUM).withTimeout(7),
      new BalanceChargeStationCommand(drivetrainSubsystem)
    );
  }

  public static CommandBase ScoreOnlyAuto(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
    return Commands.sequence(
      new OrientUpwardCommand(elevatorSubsystem),
      new ExtendArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new WaitCommand(1),
      new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX).withTimeout(2),
      new RetractArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new ConstantDriveCommand(drivetrainSubsystem, -Constants.MotorSpeedValues.LOW, 0).withTimeout(2)
    );
  }

  public static CommandBase IdleAuto() {
    return new WaitCommand(0);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
