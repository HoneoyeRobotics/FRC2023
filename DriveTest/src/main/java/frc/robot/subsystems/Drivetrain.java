// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveCurrentLimits;

public class Drivetrain extends SubsystemBase {
  private double LFmaxcurrent = 0;
  private double LRmaxcurrent = 0;
  private double RFmaxcurrent = 0;
  private double RRmaxcurrent = 0;
  private int i;

  private CANSparkMax leftFrontDrive;
  private CANSparkMax leftRearDrive;
  private CANSparkMax rightFrontDrive;
  private CANSparkMax rightRearDrive;

  private MotorControllerGroup rightMotors;
  private MotorControllerGroup leftMotors;

  private DifferentialDrive drive;
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    leftFrontDrive = new CANSparkMax(31, MotorType.kBrushless);
    leftRearDrive = new CANSparkMax(34, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(32, MotorType.kBrushless);
    rightRearDrive = new CANSparkMax(33, MotorType.kBrushless);

    leftFrontDrive.setInverted(false);
    leftRearDrive.setInverted(false);
    rightFrontDrive.setInverted(true);
    rightRearDrive.setInverted(true);

    leftFrontDrive.setIdleMode(IdleMode.kBrake);
    leftRearDrive.setIdleMode(IdleMode.kBrake);
    rightFrontDrive.setIdleMode(IdleMode.kBrake);
    rightRearDrive.setIdleMode(IdleMode.kBrake);

    
    // leftFrontDrive.setSmartCurrentLimit(DriveCurrentLimits.StallCurrentLimit, DriveCurrentLimits.FreeCurrentLimit);
    // leftRearDrive.setSmartCurrentLimit(DriveCurrentLimits.StallCurrentLimit, DriveCurrentLimits.FreeCurrentLimit);
    // rightFrontDrive.setSmartCurrentLimit(DriveCurrentLimits.StallCurrentLimit, DriveCurrentLimits.FreeCurrentLimit);
    // rightRearDrive.setSmartCurrentLimit(DriveCurrentLimits.StallCurrentLimit, DriveCurrentLimits.FreeCurrentLimit);

    // leftFrontDrive.setSecondaryCurrentLimit(DriveCurrentLimits.SecondaryCurrentLimit);
    // leftRearDrive.setSecondaryCurrentLimit(DriveCurrentLimits.SecondaryCurrentLimit);
    // rightFrontDrive.setSecondaryCurrentLimit(DriveCurrentLimits.SecondaryCurrentLimit);
    // rightRearDrive.setSecondaryCurrentLimit(DriveCurrentLimits.SecondaryCurrentLimit);


    leftMotors = new MotorControllerGroup(leftFrontDrive, leftRearDrive);
    rightMotors = new MotorControllerGroup(rightFrontDrive, rightRearDrive);

    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void arcadeDrive(double xspeed, double zrotation) {
    drive.arcadeDrive(xspeed, zrotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double LFC = leftFrontDrive.getOutputCurrent();
    double LRC = leftRearDrive.getOutputCurrent();
    double RFC = rightFrontDrive.getOutputCurrent();
    double RRC = rightRearDrive.getOutputCurrent();

    if(leftFrontDrive.getOutputCurrent() < .1)
      i = 0;
    if(leftFrontDrive.getOutputCurrent() != 0)
      i++;

      
    if(i >= 15) {
      LFmaxcurrent = (LFmaxcurrent < LFC ? LFC : LFmaxcurrent);
      LRmaxcurrent = (LRmaxcurrent < LRC ? LRC : LRmaxcurrent);
      RFmaxcurrent = (RFmaxcurrent < RFC ? RFC : RFmaxcurrent);
      RRmaxcurrent = (RRmaxcurrent < RRC ? RRC : RFmaxcurrent);
    }




    SmartDashboard.putNumber("LFcurrent", LFC);
    SmartDashboard.putNumber("LRcurrent", LRC);
    SmartDashboard.putNumber("RFcurrent", RFC);
    SmartDashboard.putNumber("RRcurrent", RRC);
    SmartDashboard.putNumber("LFmax", LFmaxcurrent);
    SmartDashboard.putNumber("LRmax", LRmaxcurrent);
    SmartDashboard.putNumber("RFmax", RFmaxcurrent);
    SmartDashboard.putNumber("RRmax", RRmaxcurrent);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
