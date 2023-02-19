// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
  private final PWMVictorSPX motor;

  public GripperSubsystem() {
    motor = new PWMVictorSPX(Constants.DriverPorts.GRIPPER_MAIN);
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }
}