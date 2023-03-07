// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.commands.Autos;
import frc.robot.commands.CenterTargetCommand;
import frc.robot.commands.drive.BalanceChargeStationCommand;
import frc.robot.commands.drive.ConstantDriveCommand;
import frc.robot.commands.drive.DriveAngleCommand;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.commands.drive.TurnAngleCommand;
import frc.robot.commands.elevator.presets.OrientUpwardCommand;
import frc.robot.commands.elevator.presets.OrientDownwardCommand;
import frc.robot.commands.elevator.presets.OrientFlatCommand;
import frc.robot.commands.elevator.presets.OrientTargetCommand;
import frc.robot.commands.elevator.CalibrateElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.arm.ExtendArmCommand;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.RetractArmCommand;
import frc.robot.commands.RunGripperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);
  private final CommandXboxController controller = new CommandXboxController(OperatorConstants.CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public boolean bypassElevatorSafety = false;

  public RobotContainer() {
    configureButtonBindings();
    configureCommands();
    configureDashboard();
  }

  private void configureButtonBindings() {    
    // Joystick
    joystick.button(3).toggleOnTrue(new BalanceChargeStationCommand(drivetrainSubsystem));
    joystick.button(5).toggleOnTrue(new DriveAngleCommand(drivetrainSubsystem, 90));

    joystick.trigger().whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem, Constants.Vision.PIPELINE_REFLECTIVE));
    joystick.button(8).whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem, Constants.Vision.PIPELINE_APRILTAG));

    joystick.button(6).whileTrue(new MoveElevatorCommand(elevatorSubsystem, () -> -Constants.MotorSpeedValues.HIGH, () -> 0)); // FRONT UP
    joystick.button(5).whileTrue(new MoveElevatorCommand(elevatorSubsystem, () -> Constants.MotorSpeedValues.MAX, () -> 0)); // FRONT DOWN
    joystick.button(9).whileTrue(new MoveElevatorCommand(elevatorSubsystem, () -> 0, () -> -Constants.MotorSpeedValues.HIGH)); // REAR UP
    joystick.button(10).whileTrue(new MoveElevatorCommand(elevatorSubsystem, () -> 0, () -> Constants.MotorSpeedValues.HIGH)); // REAR DOWN

    joystick.button(11).whileTrue(new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX)); // CONE INTAKE
    joystick.button(12).whileTrue(new RunGripperCommand(gripperSubsystem, -Constants.MotorSpeedValues.MAX)); // CUBE INTAKE

    joystick.button(15).whileTrue(new MoveArmCommand(armSubsystem, () -> -controller.getRightTriggerAxis())); // EXTEND
    joystick.button(16).whileTrue(new MoveArmCommand(armSubsystem, () -> controller.getRightTriggerAxis())); // RETRACT


    // 🚧 TESTING | WILL BE REMOVED 🚧
    // ------------------------------------------------------------------------------------------------------------
      joystick.povUp().onTrue(new TurnAngleCommand(drivetrainSubsystem, 0));
      joystick.povUpRight().onTrue(new TurnAngleCommand(drivetrainSubsystem, 45));
      joystick.povRight().onTrue(new TurnAngleCommand(drivetrainSubsystem, 90));
      joystick.povDownRight().onTrue(new TurnAngleCommand(drivetrainSubsystem, 135));
      joystick.povDown().onTrue(new TurnAngleCommand(drivetrainSubsystem, 180));
      joystick.povDownLeft().onTrue(new TurnAngleCommand(drivetrainSubsystem, -135));
      joystick.povLeft().onTrue(new TurnAngleCommand(drivetrainSubsystem, -90));
      joystick.povUpLeft().onTrue(new TurnAngleCommand(drivetrainSubsystem, -45));
    // ------------------------------------------------------------------------------------------------------------

    // ✅ CORRECT BINDINGS ✅
    // joystick.povLeft().whileTrue(new ConstantDriveCommand(drivetrainSubsystem, 0, -Constants.MotorSpeedValues.LOW));
    // joystick.povRight().whileTrue(new ConstantDriveCommand(drivetrainSubsystem, 0, Constants.MotorSpeedValues.LOW));

    // Controller
    controller.rightTrigger().whileTrue(new MoveArmCommand(armSubsystem, () -> -controller.getRightTriggerAxis())); // EXTEND
    controller.leftTrigger().whileTrue(new MoveArmCommand(armSubsystem, () -> controller.getLeftTriggerAxis())); // RETRACT

    controller.rightBumper().whileTrue(new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX)); // CONE INTAKE
    controller.leftBumper().whileTrue(new RunGripperCommand(gripperSubsystem, -Constants.MotorSpeedValues.MAX)); // CUBE INTAKE
  
    controller.povLeft().onTrue(new RetractArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX));
    controller.povRight().onTrue(new ExtendArmCommand(armSubsystem, Constants.MotorSpeedValues.MAX));

    controller.a().onTrue(new OrientDownwardCommand(elevatorSubsystem));
    controller.b().onTrue(new OrientUpwardCommand(elevatorSubsystem));
    controller.x().onTrue(new OrientFlatCommand(elevatorSubsystem));
    controller.y().onTrue(new CalibrateElevatorCommand(elevatorSubsystem).andThen(new OrientTargetCommand(elevatorSubsystem)));
    // controller.y().onTrue(new ExtendArmCommand(armSubsystem).andThen(new RunGripperCommand(gripperSubsystem, Constants.MotorSpeedValues.MAX).withTimeout(2)));

    controller.start().toggleOnTrue(Commands.startEnd(() -> elevatorSubsystem.bypassSafety(true), () -> elevatorSubsystem.bypassSafety(false), elevatorSubsystem));
  }

  private void configureCommands() {
    // Setting up the auto chooser
    autoChooser.addOption("Score & Balance", Autos.ScoreBalanceAuto(drivetrainSubsystem, elevatorSubsystem, armSubsystem, gripperSubsystem));
    autoChooser.addOption("Score Only", Autos.ScoreOnlyAuto(drivetrainSubsystem, elevatorSubsystem, armSubsystem, gripperSubsystem));
    autoChooser.addOption("Balance Only", Autos.BalanceOnlyAuto(drivetrainSubsystem));
    autoChooser.setDefaultOption("Idle", Autos.IdleAuto());

    // Setting the default commands of subsystems
    drivetrainSubsystem.setDefaultCommand(new DynamicDriveCommand(drivetrainSubsystem, () -> joystick.getY() * (joystick.getRawAxis(3)+1)/2, () -> joystick.getZ() * (joystick.getRawAxis(3)+1)/2));
    elevatorSubsystem.setDefaultCommand(new MoveElevatorCommand(elevatorSubsystem, controller::getLeftY, controller::getRightY));
  }

  private void configureDashboard() {
    SmartDashboard.putData(autoChooser);

    /*  
        Logs
          - Is Balanced | BalanceChargeStationCommand (boolean)
          - Arm Switch | ArmSubsystem (boolean)
          - Front Bottom Switch | ElevatorSubsystem (boolean)
          - Front Top Switch | ElevatorSubsystem (boolean)
          - Rear Bottom Switch | ElevatorSubsystem (boolean)
          - Rear Top Switch | ElevatorSubsystem (boolean)
          - Encoder Raw | ElevatorSubsystem (number)

          - Auto Picker | RobotContainer (SendableChooser)
    */  
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
