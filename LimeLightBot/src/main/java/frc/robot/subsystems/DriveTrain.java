// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(1) ;
  
  private WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(8) ;
  
  private WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(7) ;
  
  private WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(2) ;

  private MotorControllerGroup leftDrives ;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive drive;


  
  public DriveTrain() {

frontLeftMotor.setInverted(true);
rearLeftMotor.setInverted(true);

    leftDrives = new MotorControllerGroup(frontLeftMotor, rearLeftMotor);
    rightDrives = new MotorControllerGroup(frontRightMotor, rearRightMotor);
    drive = new DifferentialDrive(leftDrives, rightDrives);

  }


  public void drive(double xSpeed, double zRotation){
    SmartDashboard.putNumber("drive:xSpeed", xSpeed);
    SmartDashboard.putNumber("drive:zRoation", zRotation);
    drive.arcadeDrive(xSpeed, zRotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
