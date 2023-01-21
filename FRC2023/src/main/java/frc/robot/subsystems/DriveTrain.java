// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;
  private DifferentialDriveKinematics  m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  private AHRS navx;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(Constants.CanIDs.LeftFrontDrive, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(Constants.CanIDs.LeftRearDrive, MotorType.kBrushless);
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);

    rightFrontMotor = new CANSparkMax(Constants.CanIDs.RightFrontDrive, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(Constants.CanIDs.RightRearDrive, MotorType.kBrushless);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftFrontMotor.getEncoder().setPosition(0);    
    leftRearMotor.getEncoder().setPosition(0);
    rightFrontMotor.getEncoder().setPosition(0);    
    rightRearMotor.getEncoder().setPosition(0);
    double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (0.1524 * Math.PI) / (double) 10.4;

    leftFrontMotor.getEncoder().setVelocityConversionFactor(kEncoderDistancePerPulse);
    
    rightFrontMotor.getEncoder().setVelocityConversionFactor(kEncoderDistancePerPulse);
    navx = new AHRS(SerialPort.Port.kUSB);
    //reset navx when robot boots
    navx.reset();
    
    m_kinematics = new DifferentialDriveKinematics(29.625 / 39.37);
    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());
  }

  public void drive(double xspeed, double zrotation) {
    drive.arcadeDrive(xspeed, zrotation);
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.update(navx.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition());
  

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

  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFrontMotor.getEncoder().getVelocity(), rightFrontMotor.getEncoder().getVelocity());
  }
  public void resetEncoders(){
    leftFrontMotor.getEncoder().setPosition(0);
    rightFrontMotor.getEncoder().setPosition(0);
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navx.getRotation2d(), leftFrontMotor.getEncoder().getPosition(), rightFrontMotor.getEncoder().getPosition(), pose);
  }


  public double getAverageEncoderDistance() {
    return (leftFrontMotor.getEncoder().getPosition() +rightFrontMotor.getEncoder().getPosition()) / 2.0;
  }
  public void zeroHeading() {
    navx.reset();
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -navx.getRate();
  }

  private final double KS = 2;
  private final double KV = 2;
  private final double KA = 1;

  public double getOutputVolts(){
    return leftFrontMotor.getBusVoltage();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }
  
  public Command FollowPath(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(KS, KV, KA),
            m_kinematics,
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
}

}
