// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveTrain;

public class TeleopDrive extends CommandBase {
  private DriveTrain m_drivetrain;
  private DoubleSupplier m_leftTriggerSupplier;
  private DoubleSupplier m_rightTriggerSupplier;
  private DoubleSupplier m_leftStickXSupplier;

  //private double oldSpeed = 0.0;
  private double xSpeed;
  private double zRotation;
  private double tmpSpeed;

  public TeleopDrive(DriveTrain drivetrain, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier,
      DoubleSupplier leftStickXSupplier) {
    m_leftTriggerSupplier = leftTriggerSupplier;
    m_rightTriggerSupplier = rightTriggerSupplier;
    m_leftStickXSupplier = leftStickXSupplier;

    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = (m_rightTriggerSupplier.getAsDouble() - m_leftTriggerSupplier.getAsDouble());
    zRotation = (m_leftStickXSupplier.getAsDouble());

    // xSpeed = xSpeed * xSpeed * (xSpeed > 0 ? 1 : -1);
    tmpSpeed = xSpeed * xSpeed;
    if (xSpeed < 0)
       xSpeed = tmpSpeed * -1.0;
    else
       xSpeed = tmpSpeed;
      
   zRotation = zRotation * zRotation * (zRotation > 0 ? 1 : -1);
    // if (m_drivetrain.getFast() == false) {
    //   xSpeed = xSpeed / 1.75;
    //   zRotation = zRotation / 2;
    // }
    if ((zRotation < Drive.deadband) && (zRotation > (Drive.deadband * -1)))
      zRotation = 0;

    // if (xSpeed > 0)
    //   xSpeed = (xSpeed > oldSpeed + Drive.ramprate ? oldSpeed + Drive.ramprate : xSpeed);
    // if (xSpeed < 0)
    //   xSpeed = (xSpeed < oldSpeed - Drive.ramprate ? oldSpeed - Drive.ramprate : xSpeed);
    if ((xSpeed < Drive.deadband) && (xSpeed > (Drive.deadband * -1)))
      xSpeed = 0;

    m_drivetrain.arcadeDrive(xSpeed, zRotation * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
