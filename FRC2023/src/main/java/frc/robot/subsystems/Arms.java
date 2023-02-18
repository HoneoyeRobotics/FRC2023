// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.ArmRotate;

public class Arms extends SubsystemBase {
  /** Creates a new Arms. */
  private boolean clawOpened=false;
  private final DoubleSolenoid clawSolenoid;
  private final TalonSRX armLengthMotor;
  private final CANSparkMax armRotateMotor;
  private final DoubleSolenoid armLengthBrakeSolenoid;
  private final DoubleSolenoid armRotateBrakeSolenoid;
  private boolean armLengthBrake = false;
  private boolean armRotateBrake = false;
  private double currentPosition;

 // private final Compressor compressor;
  public Arms() {
    armLengthBrakeSolenoid = new DoubleSolenoid(Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Arm_Length_Brake_On, Constants.PCMIDs.Arm_Length_Brake_Off);
    armRotateBrakeSolenoid = new DoubleSolenoid(Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Arm_Rotate_Brake_On, Constants.PCMIDs.Arm_Rotate_Brake_Off);
    
    clawSolenoid = new DoubleSolenoid (Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Claw_Forward, Constants.PCMIDs.Claw_Reverse);    
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    armLengthMotor = new TalonSRX(Constants.CanIDs.ArmLengthMotor);

    armRotateMotor = new CANSparkMax(Constants.CanIDs.ArmRotateMotor, MotorType.kBrushless);
    armRotateMotor.setInverted(true);
  }

  public boolean isArmLengthBrakeOn(){
    return armLengthBrake;
  }

  public void armLengthBrakeOn(){
    armLengthBrakeSolenoid.set(DoubleSolenoid.Value.kForward);
    armLengthBrake = true;
  }

  public void armLenghtBrakeOff(){
    armLengthBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    armLengthBrake = false;
  }


  public boolean isArmRotateBrakeOn(){
    return armRotateBrake;
  }

  public void armRotateBrakeOn(){
    armRotateBrakeSolenoid.set(DoubleSolenoid.Value.kForward);
    armRotateBrake = true;
  }

  public void armRotateBrakeOff(){
    armRotateBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    armRotateBrake = false;
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

  public double armLengthMotorCurrentPosition() {
    return armLengthMotor.getSelectedSensorPosition();
  }

  public void moveArmInOut(double speed){
    if(speed < 0 && armLengthMotor.getSelectedSensorPosition() <= 0)
      speed = 0;
    if(speed > 0 && armLengthMotor.getSelectedSensorPosition() >= RobotPrefs.getArmLengthMax())
      speed = 0;
    armLengthMotor.set(ControlMode.PercentOutput, speed);
  }

  public void moveArmToPosition(double speed, double position) {
    currentPosition = armRotateMotor.getEncoder().getPosition();
    if (position > currentPosition - ArmLength.deadband && position < currentPosition + ArmLength.deadband) {
      speed = 0;
    }

    armRotateMotor.set(speed);
  }

  public  void resetArmRotateEncoder() {
    armRotateMotor.getEncoder().setPosition(0);
  }

  public double armRotateMotorCurrentPosition() {
    return armRotateMotor.getEncoder().getPosition();
  }

  public void rotateArmToPosition(double speed, double position) {
    currentPosition = armRotateMotor.getEncoder().getPosition();
    if (position > currentPosition - ArmRotate.deadband && position < currentPosition + ArmRotate.deadband) {
      speed = 0;
    }

    armRotateMotor.set(speed);
  }


  public void rotateArm(double speed) {
    armRotateMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("clawOpened", clawOpened);
    SmartDashboard.putNumber("Arm Length Encoder", armLengthMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ArmRotatePosition", armRotateMotor.getEncoder().getPosition());
  }
}
