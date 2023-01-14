// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(26, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(25, MotorType.kBrushless);
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);

    rightFrontMotor = new CANSparkMax(21, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(22, MotorType.kBrushless);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftFrontMotor.getEncoder().setPosition(0);    
    leftRearMotor.getEncoder().setPosition(0);
    rightFrontMotor.getEncoder().setPosition(0);    
    rightRearMotor.getEncoder().setPosition(0);
  
  }

  public void drive(double xspeed, double zrotation) {
    drive.arcadeDrive(xspeed, zrotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LF Encoder", leftFrontMotor.getEncoder().getPosition());    
    SmartDashboard.putNumber("LR Encoder", leftRearMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RF Encoder", rightFrontMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RR Encoder", rightRearMotor.getEncoder().getPosition());
  }
}
