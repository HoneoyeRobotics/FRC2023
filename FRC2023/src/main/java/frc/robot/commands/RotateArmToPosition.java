// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Arms;
import frc.robot.Constants.ArmRotate;

public class RotateArmToPosition extends CommandBase {
  private Arms m_arms;
  private Double m_position;
  private double speed;
  /** Creates a new RotateArmToPosition. */
  public RotateArmToPosition(Arms arms, double position) {
    m_arms = arms;
    m_position = position;
    addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arms.armRotateBrakeOff();
    if(m_arms.isArmRotateIPDEnabled() == false){

    
    if(m_position > m_arms.armRotateMotorCurrentPosition())
      speed = RobotPrefs.getArmRotateUpSpeed();
    else
      speed = RobotPrefs.getArmRotateDownSpeed();
    }
    else{
      m_arms.moveArmRotatePIDPosition(0, true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arms.rotateArmToPosition(speed, m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_arms.isArmRotateIPDEnabled() == false)
    m_arms.rotateArmToPosition(0.0, (m_arms.armRotateMotorCurrentPosition()));
    // m_arms.armRotateBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_arms.armRotateMotorCurrentPosition() + ArmRotate.deadband) > m_position && 
            (m_arms.armRotateMotorCurrentPosition() - ArmRotate.deadband) < m_position) || m_arms.isArmRotateIPDEnabled();
  }
}