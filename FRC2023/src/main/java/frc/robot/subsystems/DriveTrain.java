// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;

  private AHRS navx;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(Constants.CanIDs.LeftFrontDrive, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(Constants.CanIDs.LeftRearDrive, MotorType.kBrushless);
   
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);

    rightFrontMotor = new CANSparkMax(Constants.CanIDs.RightFrontDrive, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(Constants.CanIDs.RightRearDrive, MotorType.kBrushless);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
    
    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftFrontMotor.getEncoder().setPosition(0);    
    leftRearMotor.getEncoder().setPosition(0);
    rightFrontMotor.getEncoder().setPosition(0);    
    rightRearMotor.getEncoder().setPosition(0);
  

    navx = new AHRS(SerialPort.Port.kUSB);
    //reset navx when robot boots
    navx.reset();
  }

  public void arcadeDrive(double xspeed, double zrotation) {
    drive.arcadeDrive(xspeed, zrotation);
  }

  public void tankDrive(double left, double right)
  {
    drive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(RobotPrefs.getDebugMode()){
      //only show in debug mode set with the robot preferences.
      SmartDashboard.putNumber("Rotation", navx.getAngle());
      SmartDashboard.putNumber("Roll (LR tip)", navx.getRoll());
      SmartDashboard.putNumber("Pitch (FR tip)", navx.getPitch());
      SmartDashboard.putNumber("LF Encoder", leftFrontMotor.getEncoder().getPosition());    
      SmartDashboard.putNumber("LR Encoder", leftRearMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("RF Encoder", rightFrontMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("RR Encoder", rightRearMotor.getEncoder().getPosition());

    }
  }
}
