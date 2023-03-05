// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToScorePos extends SequentialCommandGroup {
  private DriveTrain m_driveTrain;
  private Vision m_vision;
  private Arms m_arms;
  /** Creates a new MoveToScorePos. */
  public MoveToScorePos(DriveTrain driveTrain, Vision vision, Arms arms) {
    m_driveTrain = driveTrain;
    m_vision = vision;
    m_arms = arms;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveUntilCorrectDistance(m_driveTrain, m_vision),
      new RotateToDegree(m_driveTrain, m_driveTrain.isBlue() ? 90 : -90),
      new DriveUntilPerpendicular(m_driveTrain, m_vision, m_arms),
      new RotateToDegree(m_driveTrain, 0),
      new DriveUntilCollision(driveTrain)
    );
  }
}
