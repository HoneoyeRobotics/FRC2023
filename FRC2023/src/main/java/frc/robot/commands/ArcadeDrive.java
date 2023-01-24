// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.AbstractDocument.BranchElement;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.DriveTrain;
import frc.robot.enums.*;

public class ArcadeDrive extends CommandBase {
  private DriveTrain drivetrain;
  private DoubleSupplier leftTriggerSupplier;
  private DoubleSupplier rightTriggerSupplier;
  private DoubleSupplier leftStickXSupplier;
  private DoubleSupplier leftStickYSupplier;
  private BooleanSupplier slowSupplier;
  private DoubleSupplier rightStickYSupplier;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain drivetrain, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier, DoubleSupplier leftStickXSupplier, DoubleSupplier leftStickYSupplier, BooleanSupplier slowSupplier, DoubleSupplier rightStickYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.rightStickYSupplier = rightStickYSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.leftStickXSupplier = leftStickXSupplier;
    this.leftStickYSupplier = leftStickYSupplier;
    this.slowSupplier = slowSupplier;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double xSpeed = 0;
    double zRotation = 0;
    double tempZ = 0;

    DriveControlType controlType = RobotPrefs.getDriveControlType();
    SmartDashboard.putString("controlType", controlType.toString());
    SmartDashboard.putNumber("leftx", leftStickXSupplier.getAsDouble());
    SmartDashboard.putNumber("lefty", leftStickYSupplier.getAsDouble());
    SmartDashboard.putNumber("lefttrigger", leftTriggerSupplier.getAsDouble());
    SmartDashboard.putNumber("righttrigger", rightTriggerSupplier.getAsDouble());
    
    //get preferences based on drive type
    switch(controlType){
      case TriggersForward:
        xSpeed = rightTriggerSupplier.getAsDouble() - leftTriggerSupplier.getAsDouble();
        tempZ = leftStickXSupplier.getAsDouble();
        break;
      case TriggersTurn:
        xSpeed = leftStickYSupplier.getAsDouble() * -1;
        tempZ = rightTriggerSupplier.getAsDouble() - leftTriggerSupplier.getAsDouble();
        break;
      case TriggersTurnDoubleForward:      
          xSpeed = (leftStickYSupplier.getAsDouble() + rightStickYSupplier.getAsDouble()) / 2 * -1;
          tempZ = rightTriggerSupplier.getAsDouble() - leftTriggerSupplier.getAsDouble();
        break;
      case SingleStick:
      default:
        xSpeed = leftStickYSupplier.getAsDouble() * -1;
        tempZ = leftStickXSupplier.getAsDouble();
        break;
    }

 
    tempZ = tempZ * .8;
    zRotation = tempZ * tempZ * ((tempZ < 0) ? -1 : 1);
    
    // if(drivetrain.getReverse() == true)
    // xSpeed = xSpeed * -1;
    if(slowSupplier.getAsBoolean()){
      xSpeed *= 0.1;
      zRotation *= 0.5;
    }
    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("zrotation", zRotation);
    drivetrain.drive(xSpeed, zRotation); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
