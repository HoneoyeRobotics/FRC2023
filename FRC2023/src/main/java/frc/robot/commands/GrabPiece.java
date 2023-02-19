// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.ArmRotate;
import frc.robot.enums.GrabPosition;
import frc.robot.subsystems.Arms;

public class GrabPiece extends CommandBase {
  private Arms arms;
  private GrabPosition grabPosition;
  private double rotateSpeed;
  private double lengthSpeed;
  private double rotatePosition;
  private double lengthPosition;  

  /** Creates a new GrabPiece. */
  public GrabPiece(Arms m_arms) {
    arms = m_arms;
    addRequirements(arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateSpeed = RobotPrefs.getArmRotateUpSpeed();
    lengthSpeed = RobotPrefs.getArmLengthOutSpeed();

    switch(grabPosition){
      case Cube:
        rotatePosition = RobotPrefs.getArmRotateGrabCube();
        lengthPosition = RobotPrefs.getArmLengthGrabCube();
        break;
      case ConePointIn:
      rotatePosition = RobotPrefs.getArmRotateGrabConePointIn();
      lengthPosition = RobotPrefs.getArmLengthGrabConePointIn();
        break;
      case ConePointUp:
      rotatePosition = RobotPrefs.getArmRotateGrabConePointUp();
      lengthPosition = RobotPrefs.getArmLengthGrabConePointUp();
        break;
      case ConePointOut:
      rotatePosition = RobotPrefs.getArmRotateGrabConePointOut();
      lengthPosition = RobotPrefs.getArmLengthGrabConePointOut();
        break;
      default:
        break;
    }
    arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.rotateArmToPosition(rotateSpeed, rotatePosition);
    arms.moveArmToPosition(lengthSpeed, lengthPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.rotateArmToPosition(0.0, arms.armRotateMotorCurrentPosition());
    arms.moveArmToPosition(0.0, arms.armLengthMotorCurrentPosition());
    arms.armLengthBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((arms.armLengthMotorCurrentPosition() + ArmLength.deadband) > lengthPosition && 
             (arms.armLengthMotorCurrentPosition() - ArmLength.deadband) < lengthPosition) 
             &&
            ((arms.armRotateMotorCurrentPosition() + ArmRotate.deadband) > rotatePosition && 
             (arms.armRotateMotorCurrentPosition() - ArmRotate.deadband) < rotatePosition));
  }
}
