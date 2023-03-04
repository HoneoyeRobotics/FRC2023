// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arms extends SubsystemBase {
  private AnalogInput proxSensor;
  //private DigitalInput proxSensor;
  /** Creates a new Arm. */
  public Arms() {
    proxSensor = new AnalogInput(Constants.proxSensorID);
    //proxSensor = new DigitalInput(Constants.proxSensorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ProxSensorAnalog", proxSensor.getVoltage());
    //SmartDashboard.putBoolean("ProxSensor", proxSensor.get());
  }
}
