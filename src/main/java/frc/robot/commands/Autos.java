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
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.elevator.SetRearElevatorPositionCommand;
import frc.robot.commands.elevator.presets.OrientTargetCommand;
import frc.robot.commands.elevator.presets.OrientUpwardCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  public static CommandBase ScoreOnlyAuto(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
    return Commands.sequence(
      new SetRearElevatorPositionCommand(elevatorSubsystem, 3000),
      new MoveElevatorCommand(elevatorSubsystem, () -> -0.3, () -> -1).withTimeout(1.9),
      new ExtendArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX).withTimeout(1.5),
      new MoveElevatorCommand(elevatorSubsystem, () -> 0, () -> 1).withTimeout(1),
      Commands.parallel(new MoveElevatorCommand(elevatorSubsystem, () -> -0.25, () -> 0).withTimeout(5),
      new RetractArmCommand(armSubsystem))
    );
  }

  public static CommandBase ScoreAndExitAuto(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
    return Commands.sequence(
      new SetRearElevatorPositionCommand(elevatorSubsystem, 3000),
      new MoveElevatorCommand(elevatorSubsystem, () -> -0.3, () -> -1).withTimeout(1.9),
      new ExtendArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX),
      new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX).withTimeout(1.5)
   
    );
  }

  public static CommandBase BalanceOnlyAuto(DrivetrainSubsystem drivetrainSubsystem) {
    return Commands.sequence(
      new DriveUntilChargeStationCommand(drivetrainSubsystem, -0.55).withTimeout(7),
      new BalanceChargeStationCommand(drivetrainSubsystem)
    );
  }

  public static CommandBase ExitCableCommunity(DrivetrainSubsystem drivetrainSubsystem) {
    return new ConstantDriveCommand(drivetrainSubsystem, -0.75, 0).withTimeout(2.5);
  }

  public static CommandBase ExitCommunity(DrivetrainSubsystem drivetrainSubsystem) {
    return new ConstantDriveCommand(drivetrainSubsystem, -0.6, 0).withTimeout(3);
  }

  public static CommandBase ExitCommunityClose(DrivetrainSubsystem drivetrainSubsystem) {
    return new ConstantDriveCommand(drivetrainSubsystem, -0.5, 0).withTimeout(1.5);
  }

  public static CommandBase IdleAuto() {
    return new WaitCommand(0);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
