// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.DriveTrain;

public class RotateToDegree extends CommandBase {
  private DriveTrain m_drivetrain;
  private PIDController pidController;
  private double PIDrotation;
  private double targetHeading;
  /** Creates a new RotateToDegree. */
  public RotateToDegree(DriveTrain drivetrain, double targetHeading) {
    this.targetHeading = targetHeading;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    

    pidController = new PIDController(RobotPrefs.getRotateP(), RobotPrefs.getRotateI(), 0);
    pidController.setTolerance(5,10);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {} 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double PIDrotation = pidController.calculate(m_drivetrain.getAngle(), targetHeading) * -1;
    // if(PIDrotation > -.3) {
    //   PIDrotation = -.3;
    // 
    PIDrotation = (targetHeading - m_drivetrain.getAngle() < 20) ? -.3 : -.6;
    m_drivetrain.arcadeDrive(0, PIDrotation);
    SmartDashboard.putNumber("rotatePIDvalue", PIDrotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getAngle() >= (targetHeading - 4);
  }
}
