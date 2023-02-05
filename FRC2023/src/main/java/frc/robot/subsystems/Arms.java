// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;

public class Arms extends SubsystemBase {
  /** Creates a new Arms. */
  private boolean clawOpened=false;
  private final DoubleSolenoid clawSolenoid;
  private final TalonSRX armLengthMotor;

 // private final Compressor compressor;
  public Arms() {
    clawSolenoid = new DoubleSolenoid (Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Claw_Forward, Constants.PCMIDs.Claw_Reverse);    
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    armLengthMotor = new TalonSRX(Constants.CanIDs.ArmLengthMotor);

    
  }

  public boolean isClawOpened(){
    return clawOpened;
  }

  public void openClaw(){
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    clawOpened = true;
  }
  
  public void closeClaw(){
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    clawOpened = false;
  }

  public void resetArmLengthEncoder(){
    armLengthMotor.setSelectedSensorPosition(0);
  }

  public void moveArmInOut(double speed){
    if(speed < 0 && armLengthMotor.getSelectedSensorPosition() <= 0)
      speed = 0;
    if(speed > 0 && armLengthMotor.getSelectedSensorPosition() >= RobotPrefs.getArmLengthMax())
      speed = 0;
    armLengthMotor.set(ControlMode.PercentOutput, speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("clawOpened", clawOpened);
    SmartDashboard.putNumber("Arm Length Encoder", armLengthMotor.getSelectedSensorPosition());
  }
}
