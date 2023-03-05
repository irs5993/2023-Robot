// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final PWMVictorSPX front_motor, rear_motor; 

  private final DigitalInput front_bottom_switch, front_top_switch;
  private final DigitalInput rear_bottom_switch, rear_top_switch;

  private final Encoder encoder;

  public ElevatorSubsystem() {
    front_motor = new PWMVictorSPX(Constants.DriverPorts.ELEVATOR_FRONT);
    rear_motor = new PWMVictorSPX(Constants.DriverPorts.ELEVATOR_REAR);
    rear_motor.setInverted(true);

    front_bottom_switch = new DigitalInput(Constants.SensorPorts.Elevator.SWITCH_FRONT_BOTTOM);
    front_top_switch = new DigitalInput(Constants.SensorPorts.Elevator.SWITCH_FRONT_TOP);
    rear_bottom_switch = new DigitalInput(Constants.SensorPorts.Elevator.SWITCH_REAR_BOTTOM);
    rear_top_switch = new DigitalInput(Constants.SensorPorts.Elevator.SWITCH_REAR_TOP);

    encoder = new Encoder(Constants.SensorPorts.Elevator.ENCODER_SIGNAL_A, Constants.SensorPorts.Elevator.ENCODER_SIGNAL_B);
    // rear_encoder.setDistancePerPulse(x);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Front Bottom Switch", getFrontBottomSwitch());
    SmartDashboard.putBoolean("Front Top Switch", getFrontTopSwitch());
    SmartDashboard.putBoolean("Rear Bottom Switch", getRearBottomSwitch());
    SmartDashboard.putBoolean("Rear Top Switch", getRearTopSwitch());
    SmartDashboard.putNumber("Encoder Raw", getEncoderRaw());
  }

  public void setFrontElevatorSpeed(double speed) {
    // Safety protocol to ensure that the cargo doesn't go beyond it's frame

    // if (speed > 0) {
    //   // Intends to move upwards, prevent if the top switch is on
    //   if (front_top_switch.get()) return;
    // } else {
    //   // Intends to move downwards, prevent if the bottom switch is on
    //   if (front_bottom_switch.get()) return;
    // }

    front_motor.set(speed * 0.6);
  }

  public void setRearElevatorSpeed(double speed) {
    // if (speed > 0) {
    //   if (rear_top_switch.get()) return;
    // } else {
    //   if (rear_bottom_switch.get()) return;
    // }

    rear_motor.set(speed);
  }

  // Switch state getters
  public boolean getFrontBottomSwitch() {
    return front_bottom_switch.get();
  }

  public boolean getFrontTopSwitch() {
    return front_top_switch.get();
  }

  public boolean getRearBottomSwitch() {
    return rear_bottom_switch.get();
  }

  public boolean getRearTopSwitch() {
    return rear_top_switch.get();
  }

  // Encoder value getters
  public double getEncoderRaw() {
    return encoder.getRaw();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public void stopRear() {
    rear_motor.stopMotor();
  }

  public void stopFront() {
    front_motor.stopMotor();
  }

  public void stop() {
    stopRear();
    stopFront();
  }
}
