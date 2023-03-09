// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final PWMVictorSPX left_motor;
  private final PWMVictorSPX right_motor;

  private final DifferentialDrive drive_base;
  private final AHRS gyro;

  public DrivetrainSubsystem() {
    left_motor = new PWMVictorSPX(Constants.DriverPorts.CHASIS_LEFT);
    right_motor = new PWMVictorSPX(Constants.DriverPorts.CHASIS_RIGHT);
    right_motor.setInverted(true);

    drive_base = new DifferentialDrive(left_motor, right_motor);
    gyro = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
  }

  public void drive(double xSpeed, double zRotation) {
    drive_base.arcadeDrive(xSpeed, -zRotation);
  }

  public void driveNormal(double xSpeed, double zRotation) {
    // x -> xSpeed || zRotation
    // in_min -> 0
    // in_max -> 1
    // out_min -> Constants.RobotConstants.STALL_THRESHOLD
    // out_max -> 1

    double xNormal = Math.signum(xSpeed) * (Math.abs(xSpeed) * (1  - Constants.RobotConstants.STALL_THRESHOLD) + Constants.RobotConstants.STALL_THRESHOLD);
    double zNormal = Math.signum(zRotation) * (Math.abs(zRotation) * (1  - Constants.RobotConstants.STALL_THRESHOLD) + Constants.RobotConstants.STALL_THRESHOLD);
    
    drive_base.arcadeDrive(xNormal, zNormal);
  }
  
  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void stop() {
    drive_base.stopMotor();
  }
}
