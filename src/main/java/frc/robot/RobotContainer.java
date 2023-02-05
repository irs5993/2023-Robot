// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DynamicDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final Joystick joystick = new Joystick(OperatorConstants.JOYSTICK_PORT);

  public RobotContainer() {
    configureButtonBindings();
    configureCommands();
  }

  private void configureButtonBindings() {}

  private void configureCommands() {
    drivetrainSubsystem.setDefaultCommand(new DynamicDriveCommand(drivetrainSubsystem, joystick::getX, joystick::getY));
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(drivetrainSubsystem);
  }
}
