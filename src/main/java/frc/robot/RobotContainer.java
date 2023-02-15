// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.BalanceChargeStationCommand;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.commands.drive.FollowTargetCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final Joystick joystick = new Joystick(OperatorConstants.JOYSTICK_PORT);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    configureCommands();
    configureDashboard();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, 3).toggleOnTrue(new BalanceChargeStationCommand(drivetrainSubsystem));
    new JoystickButton(joystick, 4).toggleOnTrue(new FollowTargetCommand(drivetrainSubsystem, visionSubsystem));
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

  

  // Utility functions attached to the robot container 
  // ------------------------------------------------------------------------------------------------------------

    // Allows for a gyro reset call from Robot.java when either autonomous or the teleoperated mode is actived
    public void resetGyro() {
      drivetrainSubsystem.resetGyro();
    }

    // Allows for the gyro to finish calibration when the robotInit function is ran
    public void calibrateGyro() {
      drivetrainSubsystem.calibrateGyro();
    }

  // ------------------------------------------------------------------------------------------------------------
}
