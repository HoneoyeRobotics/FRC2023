// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous2 extends SequentialCommandGroup {
  private DriveTrain m_drivetrain;
  private Arms m_arms;
  /** Creates a new Autonomous2. */
  public Autonomous2(Arms arms, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    m_arms = arms;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GrabPiece(m_arms),
      new CloseClaw(m_arms),
      new ScorePiece(m_arms),
      new ScorePiece2(m_arms),
      new DriveUntilTipping(m_drivetrain, true).withTimeout(.25),
      new ToggleClaw(m_arms)
      );
  }
}