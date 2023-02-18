// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.subsystems.Arms;

public class MoveArmToPosition extends CommandBase {
  private Arms m_arms;
  private double speed;
  private double m_position;
  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(Arms arms, double position) {
    m_arms = arms;
    m_position = position;
    addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_position > m_arms.armLengthMotorCurrentPosition())
      speed = RobotPrefs.getArmLengthOutSpeed();
    else
      speed = RobotPrefs.getArmLengthInSpeed();

    m_arms.armLenghtBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arms.moveArmToPosition(speed, m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arms.moveArmToPosition(0.0, m_arms.armLengthMotorCurrentPosition());
    m_arms.armLengthBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_arms.armLengthMotorCurrentPosition() + ArmLength.deadband) > m_position && 
            (m_arms.armLengthMotorCurrentPosition() - ArmLength.deadband) < m_position);
  }
}
