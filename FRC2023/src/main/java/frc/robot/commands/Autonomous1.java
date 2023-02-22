// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.ScoringHeight;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous1 extends SequentialCommandGroup {

  private Arms m_arms;
  private DriveTrain m_drivetrain;
  private ScoringHeight m_scoringHeight;

  /** Creates a new Autonomous1. */
  public Autonomous1(Arms arms, DriveTrain drivetrain, ScoringHeight scoringHeight) {
    m_arms = arms;
    m_drivetrain = drivetrain;
    m_scoringHeight = scoringHeight;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScorePiece(m_arms, 5, scoringHeight.Low),
      new DriveBackwardUntilTipping(m_drivetrain),
      new BalanceOnPlatform(m_drivetrain, true)
    );
  }
}
