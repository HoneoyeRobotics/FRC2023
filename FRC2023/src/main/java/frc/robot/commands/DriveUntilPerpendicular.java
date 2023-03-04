// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveUntilPerpendicular extends CommandBase {
  private DriveTrain m_driveTrain;
  private Vision m_vision;
  private int aprilTagID = 7;
  /** Creates a new DriveUntilPerpendicular. */
  public DriveUntilPerpendicular(DriveTrain driveTrain, Vision vision) {
    m_driveTrain = driveTrain;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setBreakMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_vision.closeToPerpendicular(aprilTagID) == false)
      m_driveTrain.arcadeDrive(.5, 0);
    else 
      m_driveTrain.arcadeDrive(.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_vision.isPerpendicular(aprilTagID);
  }
}
