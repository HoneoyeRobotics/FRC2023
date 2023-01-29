// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;
  
  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(CanIDs.LeftFrontDrive, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(CanIDs.LeftRearDrive, MotorType.kBrushless);
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);

    rightFrontMotor = new CANSparkMax(CanIDs.RightFrontDrive, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(CanIDs.RightRearDrive, MotorType.kBrushless);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    if (Constants.testcode) {
  
    leftFrontMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    
    leftFrontMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
   }

    resetEncoders();
    m_odometry =
       new DifferentialDriveOdometry(
           m_gyro.getRotation2d(), leftRearMotor.getEncoder().getPosition(), rightRearMotor.getEncoder().getPosition());
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LF Encoder", leftFrontMotor.getEncoder().getPosition());    
    SmartDashboard.putNumber("LR Encoder", leftRearMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RF Encoder", rightFrontMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("RR Encoder", rightRearMotor.getEncoder().getPosition());

    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(), leftRearMotor.getEncoder().getPosition(), rightRearMotor.getEncoder().getPosition());
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
    return new DifferentialDriveWheelSpeeds(leftRearMotor.getEncoder().getVelocity(), rightRearMotor.getEncoder().getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), leftRearMotor.getEncoder().getPosition(), rightRearMotor.getEncoder().getPosition(), pose);
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
    return (leftRearMotor.getEncoder().getPosition() + rightRearMotor.getEncoder().getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANSparkMax getLeftEncoder() {
    return leftRearMotor;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANSparkMax getRightEncoder() {
    return rightRearMotor;
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


}
