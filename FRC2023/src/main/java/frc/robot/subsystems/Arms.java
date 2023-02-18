// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.ArmRotate;
import frc.robot.enums.GrabPosition;
import frc.robot.enums.ScoringHeight;

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
  private GrabPosition grabPosition = GrabPosition.Cube;
  private ScoringHeight scoringHeight = ScoringHeight.Low;
  private int scoringSlot = 1;
  private String l_position;
  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable myTable = table.getTable("Shuffleboard/Tab 2");

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

  public void armLengthBrakeOff(){
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

  public void changeGrabPosition() {
    switch(grabPosition){
      case Cube:
        grabPosition = GrabPosition.ConePointIn;
        break;
      case ConePointIn:
        grabPosition = GrabPosition.ConePointUp;
        break;
      case ConePointUp:
        grabPosition = GrabPosition.ConePointOut;
        break;
      case ConePointOut:
        grabPosition = GrabPosition.Cube;
        break;
      default:
        grabPosition = GrabPosition.Cube;
        break;
    }
    SmartDashboard.putString("GrabPosition", grabPosition.toString());
  }

  public void changeScoringHeight(boolean up) {
    switch(scoringHeight) {
      case Low: 
        if (up) 
          scoringHeight = ScoringHeight.Med;
        break;
      case Med:
        if(up) 
          scoringHeight = ScoringHeight.High;
        else 
          scoringHeight = ScoringHeight.Low;
        break;
      case High:
        if(!up) 
          scoringHeight = ScoringHeight.Med;
        break;
      default:
        scoringHeight = ScoringHeight.Med;
        break;
    }
    SmartDashboard.putString("ScoringHeight", scoringHeight.toString());
    smartDashboardScorePosition();
  }

  public int changeScoringSlot(boolean right) {
    if(right) {
      if(scoringSlot == 9)
        scoringSlot = 1;
      else 
        ++scoringSlot;
    }
    else {
      if(scoringSlot == 1) 
        scoringSlot = 9; 
      else
        --scoringSlot;
    }
    SmartDashboard.putNumber("Scoring Slot", scoringSlot);
    smartDashboardScorePosition();
    return scoringSlot;
  }

  public void smartDashboardScorePosition() {
    //NetworkTableEntry myEntry = myTable.getEntry("ScorePos1High");
    //myEntry.setBoolean(true);

    if (l_position != null) {
      myTable.getEntry(l_position).setBoolean(false);
    }
    l_position = String.format("ScorePos%d%s", scoringSlot, scoringHeight.toString());
      myTable.getEntry(l_position).setBoolean(true);
  }

  // public void updateShuffleBoard(ScoringHeight scoringHeight, int scoringSlot) {
  //     boolean ScorePos1Low;
  //     boolean ScorePos2Low;
  //     boolean ScorePos3Low;
  //     boolean ScorePos4Low;
  //     boolean ScorePos5Low;
  //     boolean ScorePos6Low;
  //     boolean ScorePos7Low;
  //     boolean ScorePos8Low;
  //     boolean ScorePos9Low;
  //     boolean ScorePos1Med;
  //     boolean ScorePos2Med;
  //     boolean ScorePos3Med;
  //     boolean ScorePos4Med;
  //     boolean ScorePos5Med;
  //     boolean ScorePos6Med;
  //     boolean ScorePos7Med;
  //     boolean ScorePos8Med;
  //     boolean ScorePos9Med;
  //     boolean ScorePos1High;
  //     boolean ScorePos2High;
  //     boolean ScorePos3High;
  //     boolean ScorePos4High;
  //     boolean ScorePos5High;
  //     boolean ScorePos6High;
  //     boolean ScorePos7High;
  //     boolean ScorePos8High;
  //     boolean ScorePos9High;

  //   if(scoringSlot == 1 && scoringHeight == scoringHeight.Low)
  //     ScorePos1Low = true;
  //   else
  //     ScorePos2Low = false;
  //   if(scoringSlot == 2 && scoringHeight == scoringHeight.Low)
  //     ScorePos2Low = true;
  //   else
  //     ScorePos2Low = false;
  //   if(scoringSlot == 3 && scoringHeight == scoringHeight.Low)
  //     ScorePos3Low = true;
  //   else
  //     ScorePos3Low = false;
  //   if(scoringSlot == 4 && scoringHeight == scoringHeight.Low)
  //     ScorePos4Low = true;
  //   else
  //     ScorePos4Low = false;
  //   if(scoringSlot == 5 && scoringHeight == scoringHeight.Low)
  //     ScorePos5Low = true;
  //   else
  //     ScorePos5Low = false;
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("clawOpened", clawOpened);
    SmartDashboard.putNumber("Arm Length Encoder", armLengthMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ArmRotatePosition", armRotateMotor.getEncoder().getPosition());
  }
}
