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
  private DoubleSupplier m_leftStickYSupplier;
  private DoubleSupplier m_rightStickYSupplier;
  private BooleanSupplier m_fastSupplier;

  private double oldSpeed;
  private double xSpeed;
  private double zRotation;

  public TeleopDrive(DriveTrain drivetrain, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier, DoubleSupplier leftStickXSupplier, DoubleSupplier leftStickYSupplier, DoubleSupplier rightStickYSupplier, BooleanSupplier fastSupplier) {
    m_drivetrain = drivetrain;
    m_leftTriggerSupplier = leftTriggerSupplier;
    m_rightTriggerSupplier = rightTriggerSupplier;
    m_leftStickXSupplier = leftStickXSupplier;
    m_leftStickYSupplier = rightStickYSupplier;
    m_rightStickYSupplier = rightStickYSupplier;
    m_fastSupplier = fastSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = (m_rightTriggerSupplier.getAsDouble() - m_leftTriggerSupplier.getAsDouble());
    zRotation = (m_leftStickXSupplier.getAsDouble());
    xSpeed = xSpeed * xSpeed * (xSpeed > 0 ? 1 : -1);
    zRotation = zRotation * zRotation * (zRotation > 0 ? 1 : -1);
    if(m_fastSupplier.getAsBoolean() == false) {
      xSpeed = xSpeed / 2;
      zRotation = zRotation / 2;
    }
    if ((zRotation < Drive.deadband) && (zRotation > (Drive.deadband * -1))) zRotation = 0;

    if (xSpeed > 0) xSpeed = (xSpeed > oldSpeed + Drive.ramprate ? oldSpeed + Drive.ramprate : xSpeed);
    if (xSpeed < 0) xSpeed = (xSpeed < oldSpeed - Drive.ramprate ? oldSpeed - Drive.ramprate : xSpeed);
    if ((xSpeed < Drive.deadband) && (xSpeed > (Drive.deadband * -1))) xSpeed = 0;

    m_drivetrain.arcadeDrive(xSpeed, zRotation);
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
