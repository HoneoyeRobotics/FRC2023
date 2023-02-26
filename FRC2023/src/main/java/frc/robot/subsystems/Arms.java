// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
  private final CANSparkMax armLengthMotor;
  private final CANSparkMax armRotateMotor;
  private final DoubleSolenoid armLengthBrakeSolenoid;
  //private final DoubleSolenoid armRotateBrakeSolenoid;
  private boolean armLengthBrake = false;
  //private boolean armRotateBrake = false;
  private double currentPosition;
  private GrabPosition grabPosition = GrabPosition.Cube;
  private ScoringHeight scoringHeight = ScoringHeight.Low;
  private int scoringSlot = 1;
  private String l_position;
  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable myTable = table.getTable("Shuffleboard/Tab 2");
  private boolean armRotatePIDEnabled = true;
  private PIDController armRotatePIDController;
  private double armRotatePIDSetpoint = 0;

  //private DigitalInput armLengthLimitSwitch;


  //todo: add limit switch for having the arm length move all the way in. also would reset the encoder when it hits the switch.

  // private final Compressor compressor;
  public Arms() {
    armLengthBrakeSolenoid = new DoubleSolenoid(Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Arm_Length_Brake_On, Constants.PCMIDs.Arm_Length_Brake_Off);
    clawSolenoid = new DoubleSolenoid (Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Claw_Forward, Constants.PCMIDs.Claw_Reverse);    
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    armLengthMotor = new CANSparkMax(Constants.CanIDs.ArmLengthMotor, MotorType.kBrushless);
    armLengthMotor.setIdleMode(IdleMode.kBrake);
    armRotateMotor = new CANSparkMax(Constants.CanIDs.ArmRotateMotor, MotorType.kBrushless);
    armRotateMotor.setInverted(true);
    armRotateMotor.setIdleMode(IdleMode.kBrake);
    armLengthMotor.setSmartCurrentLimit(40);
    //armLengthLimitSwitch = new DigitalInput(ArmLength.armLengthLimitSwitch);

    armRotatePIDController= new PIDController(Constants.ArmRotate.RotateKp, 0, 0);
    
    resetArmLengthEncoder();
    resetArmRotateEncoder();
  }

  // public boolean isArmLengthLimitSwitchOn() {
  //   return armLengthLimitSwitch.get();
  // }



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
    armLengthMotor.getEncoder().setPosition(0);
  }

  public double armLengthMotorCurrentPosition() {
    return armLengthMotor.getEncoder().getPosition();
  }

  public boolean armLengthOverload() {
    if(armLengthMotor.getMotorTemperature() > ArmLength.armLengthTempOverload){
      return true;
    } else
      return false;
  }

  public void moveArmInOut(double speed){
    //If debug is on youwill ignore the max and min values
    if(RobotPrefs.getDebugMode() == false) {
      if((speed < 0) && (armLengthMotor.getEncoder().getPosition() <= ArmLength.minPosition))
        speed = 0;
      if((speed > 0) && (armLengthMotor.getEncoder().getPosition() >= ArmLength.maxPosition)) 
        speed = 0;
    }

    SmartDashboard.putNumber("Inoutspeed", speed);
    armLengthMotor.set(speed);
  }

  public void moveArmToPosition(double speed, double position) {
    currentPosition = armLengthMotor.getEncoder().getPosition();
    if (position > currentPosition - ArmLength.deadband && position < currentPosition + ArmLength.deadband) {
      speed = 0;
    }

    armLengthMotor.set(speed);
  }

  public void toggleArmRotatePID(){
    armRotatePIDEnabled = !armRotatePIDEnabled;
    //if flipping to false, turn off motor
    if(armRotatePIDEnabled == false)
      armRotateMotor.set(0);
    else
    armRotatePIDSetpoint = armRotateMotor.getEncoder().getPosition();
  }

  public boolean isArmRotateIPDEnabled(){
    return armRotatePIDEnabled;
  }
  
  public void moveArmRotatePIDPosition(double position, boolean setPosition){
    if(setPosition)
      armRotatePIDSetpoint = position;
    else
      armRotatePIDSetpoint += position;

    //If debug mode is off this will ensure the PID does not set the position less than 0 or more than the max
    if(RobotPrefs.getDebugMode() == false) {
      if(position > ArmRotate.maxPosition)
        position = ArmRotate.maxPosition;
      if(position < ArmRotate.minPosition)
        position = ArmRotate.minPosition;
    }
  }

  public boolean isArmRotateAtPosition(){
    return armRotatePIDController.atSetpoint();
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

  public GrabPosition getGrabPosition() {
    return grabPosition;
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

  public ScoringHeight getScoringHeight() {
    return scoringHeight;
  }

  public void changeScoringSlot(boolean right) {
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
  }

  public int getScoringSlot() {
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("clawOpened", clawOpened);
    SmartDashboard.putBoolean("ArmRotatePIDEnabled", armRotatePIDEnabled);
    if(armRotatePIDEnabled){
      double rotateSpeed = armRotatePIDController.calculate(armRotateMotorCurrentPosition(), armRotatePIDSetpoint);
      armRotateMotor.set(rotateSpeed);
      SmartDashboard.putNumber("ArmRotatePIDSpeed", rotateSpeed);
    }

    SmartDashboard.putNumber("ArmLengthEncoder", armLengthMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("ArmRotateEncoder", armRotateMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("InOutTemp", armLengthMotor.getMotorTemperature());
  }
}
