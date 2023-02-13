// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final PWMVictorSPX motor;
  private final DigitalInput bottom_switch, top_switch;

  public ArmSubsystem() {
    motor = new PWMVictorSPX(Constants.DriverPorts.ARM_MAIN);
    bottom_switch = new DigitalInput(Constants.SensorPorts.Arm.SWITCH_BOTTOM);
    top_switch = new DigitalInput(Constants.SensorPorts.Arm.SWITCH_TOP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (bottom_switch.get()) return;
    } else {
      if (top_switch.get()) return;
    }

    motor.set(speed);
  }

  // Switch state getters
  public boolean getBottomSwitch() {
    return bottom_switch.get();
  }

  public boolean getTopSwitch() {
    return top_switch.get();
  }
}
