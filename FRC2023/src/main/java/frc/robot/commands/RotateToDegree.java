// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.DriveTrain;

public class RotateToDegree extends CommandBase {
  private DriveTrain m_DriveTrain;
  private double targetHeading;
  private double rotationSpeed;
  private double sign;
  private double yaw;
  private boolean clockwise;
  /** Creates a new RotateToDegree. */
  public RotateToDegree(DriveTrain driveTrain, double targetHeading) {
    m_DriveTrain = driveTrain;
    this.targetHeading = targetHeading;

    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yaw = m_DriveTrain.getYaw();
    clockwise = (targetHeading - yaw <= 0);

    sign = clockwise ? -1 : 1;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    rotationSpeed = (Math.abs(targetHeading - m_DriveTrain.getYaw()) > 15) ? RobotPrefs.getRotateRobotSpeed() : (RobotPrefs.getRotateRobotSpeed() / 2.5);
    rotationSpeed *= sign;

    m_DriveTrain.arcadeDrive(0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (clockwise ? (m_DriveTrain.getYaw() <= targetHeading) : (m_DriveTrain.getYaw() >= targetHeading));
  }
}
