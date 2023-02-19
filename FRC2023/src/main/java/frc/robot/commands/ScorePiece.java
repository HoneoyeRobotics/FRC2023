// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.ArmRotate;
import frc.robot.enums.ScoringHeight;
import frc.robot.subsystems.Arms;

public class ScorePiece extends CommandBase {
  private Arms m_arms;
  private double rotateSpeed;
  private double lengthSpeed;
  private double rotatePosition;
  private double lengthPosition;
  private int m_scoringSlot;
  private ScoringHeight m_scoringHeight;
  /** Creates a new ScorePiece. */
  public ScorePiece(Arms arms, int scoringSlot, ScoringHeight scoringHeight) {
    m_arms = arms;
    m_scoringSlot = scoringSlot;
    m_scoringHeight = scoringHeight;
    addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateSpeed = RobotPrefs.getArmRotateUpSpeed();
    lengthSpeed = RobotPrefs.getArmLengthOutSpeed();


  if(m_scoringHeight == ScoringHeight.Low) {
    rotatePosition = RobotPrefs.getArmRotateScoreLow();
    lengthPosition = RobotPrefs.getArmLengthScoreLow();
  }
  else {
    if(m_scoringHeight == ScoringHeight.Med) {
      switch(m_scoringSlot) {
        case 1: case 3: case 4: case 6: case 7: case 9:
          rotatePosition = RobotPrefs.getArmRotateScoreConeMed();
          lengthPosition = RobotPrefs.getArmLengthScoreConeMed();
          break;
        case 2: case 5: case 8:
          rotatePosition = RobotPrefs.getArmRotateScoreCubeMed();
          lengthPosition = RobotPrefs.getArmLengthScoreCubeMed();
          break;
        default:
          break;
      }
    }
    else {
      switch(m_scoringSlot) {
        case 1: case 3: case 4: case 6: case 7: case 9:
          
          rotatePosition = RobotPrefs.getArmRotateScoreConeHigh();
          lengthPosition = RobotPrefs.getArmLengthScoreConeHigh();
          break;
        case 2: case 5: case 8:
          rotatePosition = RobotPrefs.getArmRotateScoreCubeHigh();
          lengthPosition = RobotPrefs.getArmLengthScoreCubeHigh();
          break;
        default:
          break;
      }
    }
    }

    // m_arms.armRotateBrakeOff();  
    m_arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arms.rotateArmToPosition(rotateSpeed, rotatePosition);
    m_arms.moveArmToPosition(lengthSpeed, lengthPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arms.rotateArmToPosition(0.0, m_arms.armRotateMotorCurrentPosition());
    // m_arms.armRotateBrakeOn();
    m_arms.moveArmToPosition(0.0, m_arms.armLengthMotorCurrentPosition());
    m_arms.armLengthBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((m_arms.armLengthMotorCurrentPosition() + ArmLength.deadband) > lengthPosition && 
             (m_arms.armLengthMotorCurrentPosition() - ArmLength.deadband) < lengthPosition) 
             &&
            ((m_arms.armRotateMotorCurrentPosition() + ArmRotate.deadband) > rotatePosition && 
             (m_arms.armRotateMotorCurrentPosition() - ArmRotate.deadband) < rotatePosition));
  }
}
