// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Joystick joystick = new Joystick(OperatorConstants.JOYSTICK_PORT);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureCommands();
    configureDashboard();
  }

  public void resetDrive() {
    drivetrainSubsystem.resetGyro();
  }

  private void configureButtonBindings() {
  }

  private void configureCommands() {
    // Setting up the auto chooser
    autoChooser.setDefaultOption("Default Command", Autos.exampleAuto(drivetrainSubsystem));

    // Setting the default commands of subsystems
    drivetrainSubsystem.setDefaultCommand(new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ));
  }

  private void configureDashboard() {
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(drivetrainSubsystem);
  }
}
