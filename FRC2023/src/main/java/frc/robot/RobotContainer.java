// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.enums.*;
import frc.robot.subsystems.*;
public class RobotContainer {
  private DriveTrain driveTrain;
  private Arms arms;
  private Vision vision;
  private CommandXboxController driverJoystick = new CommandXboxController(0);
  private CommandJoystick buttonBoard = new CommandJoystick(1);

  public RobotContainer() {
    driveTrain = new DriveTrain();
    arms = new Arms();
    vision = new Vision();
    //default Teleop command
    driveTrain.setDefaultCommand(new TeleopDrive(driveTrain, 
    () -> driverJoystick.getLeftTriggerAxis(), 
    () -> driverJoystick.getRightTriggerAxis(), 
    () -> driverJoystick.getLeftX(), 
    () -> driverJoystick.getHID().getLeftBumper()));

    configureBindings();
    configureButtonBoard();
    initializeScorePosition();
    SmartDashboard.putData("ResetNavX", new ResetNavX(driveTrain));
  }

  private void configureBindings() {
    driverJoystick.a().onTrue(new DriveUntilPerpendicular(driveTrain, vision, arms));
    driverJoystick.rightBumper().whileTrue(new BrakeMode(driveTrain, false));
    driverJoystick.b().onTrue(new RotateToDegree(driveTrain, 0));
    driverJoystick.start().onTrue(new MoveToScorePos(driveTrain, vision, arms));
  }

  private void configureButtonBoard() {

        //Changes the scoring position grid on dashboard
        buttonBoard.axisGreaterThan(1, .5).onTrue(new ChangeScoringHeight(arms, false));
        buttonBoard.axisLessThan(1, -.5).onTrue(new ChangeScoringHeight(arms, true));
    
        buttonBoard.axisGreaterThan(0, .5).onTrue(new ChangeScoringSlot(arms, true));
        buttonBoard.axisLessThan(0, -.5).onTrue(new ChangeScoringSlot(arms, false));
  }

  private void initializeScorePosition() {
    ShuffleboardTab tab2 = Shuffleboard.getTab("Tab 2");
    int i;      
    String l_position;
    for(i = 1; i < 10; ++i) {
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.High.toString());
      tab2.add(l_position, false).withPosition(i - 1, 0);
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.Med.toString());
      tab2.add(l_position, false).withPosition(i - 1, 1);
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.Low.toString());
      tab2.add(l_position, false).withPosition(i - 1, 2);
    }
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
