// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pickup extends SubsystemBase {
  /** Creates a new Pickup. */

  private VictorSPX bottomPickupMotor;
  public Pickup() {
    bottomPickupMotor = new VictorSPX(Constants.CanIDs.BottomPickupMotor);
  }

  public void runBottomPickupMotor(double speed){
    bottomPickupMotor.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}