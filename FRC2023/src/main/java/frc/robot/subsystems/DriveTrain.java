// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
//trying to fix some stuff

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;
  private NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  private NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(CanIDs.LeftFrontDrive, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(CanIDs.LeftRearDrive, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(CanIDs.RightFrontDrive, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(CanIDs.RightRearDrive, MotorType.kBrushless);
    
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setIdleMode(IdleMode.kBrake);
    
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);


    //reset the encoders then set the sysid
    //conversion factor of not 42.
    //burn it?
    leftFrontMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    leftRearMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightRearMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    
    leftFrontMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    leftRearMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightRearMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    
  

   // Reset Gyro and encoder values before creating a DifferntialDriveOdometry object
    m_gyro.calibrate();
    resetEncoders();

    m_odometry =
       new DifferentialDriveOdometry(
           m_gyro.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());

 
  }
           
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LF Encoder", leftFrontMotor.getEncoder().getPosition());    
    SmartDashboard.putNumber("LR Encoder", leftRearMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RF Encoder", rightFrontMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RR Encoder", rightRearMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Gyro heading", m_gyro.getRotation2d().getDegrees());

    SmartDashboard.putNumber("speed", leftFrontMotor.getAppliedOutput());

    // Update the odometry (current robot Pose) in the periodic block with current gyro angle and wheel encoders
    m_odometry.update(
      m_gyro.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());


      var translation = m_odometry.getPoseMeters().getTranslation();
      m_xEntry.setNumber(translation.getX());
      m_yEntry.setNumber(translation.getY());
    }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFrontMotor.getEncoder().getVelocity(), rightFrontMotor.getEncoder().getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.calibrate();

    m_odometry.resetPosition(
        m_gyro.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition(), pose);
  }

  public void drive(double xspeed, double zrotation) {
    drive.arcadeDrive(xspeed, zrotation);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    /* 
    if (leftVolts > 2.0 ) {leftVolts = 2.0;}
    if (rightVolts > 2.0) {leftVolts = 2.0;}
    if (leftVolts < -2.0 ) {leftVolts = -2.0;}
    if (rightVolts < -2.0) {leftVolts = -2.0;}
    */
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFrontMotor.getEncoder().setPosition(0.0);
    rightFrontMotor.getEncoder().setPosition(0.0);
    leftRearMotor.getEncoder().setPosition(0.0);
    rightRearMotor.getEncoder().setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftFrontMotor.getEncoder().getPosition() + rightFrontMotor.getEncoder().getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANSparkMax getLeftEncoder() {
    return leftFrontMotor;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANSparkMax getRightEncoder() {
    return rightFrontMotor;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }




  public void togglecoastmode() {

    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftRearMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightRearMotor.setIdleMode(IdleMode.kCoast);
  }


}
