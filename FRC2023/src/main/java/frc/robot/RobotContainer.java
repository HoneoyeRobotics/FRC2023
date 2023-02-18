// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.enums.ScoringHeight;
import frc.robot.enums.ScoringSlot;
import frc.robot.subsystems.*;

public class RobotContainer {

  public ScoringHeight scoringHeight;
  public DriveTrain drivetrain;
  public Vision vision;
  public Pickup pickup;
  public Arms arms;
  public Fingers fingers;
  CommandXboxController driverJoystick = new CommandXboxController(0);

  public RobotContainer() {
    //initialize subsystems
    drivetrain = new DriveTrain();
    vision = new Vision();
    pickup = new Pickup();
    arms = new Arms();
    fingers = new Fingers();

    //wire default commands
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain,
      () -> driverJoystick.getLeftTriggerAxis(),
      () -> driverJoystick.getRightTriggerAxis(),
      () -> driverJoystick.getLeftX(),
      () -> driverJoystick.getLeftY(),
      () -> driverJoystick.getHID().getLeftBumper(),
      () -> driverJoystick.getRightY()
      ));
    initializeScorePosition();
    configureBindings();
    arms.resetArmLengthEncoder();
    arms.resetArmRotateEncoder();
    SmartDashboard.putData(new ResetArmLengthEncoder(arms));
    SmartDashboard.putData(new ResetArmRotateEncoder(arms));
    SmartDashboard.putData(new ToggleArmRotateBrake(arms));
    SmartDashboard.putData(new ToggleArmLengthBrake(arms));
  }

  private void initializeScorePosition() {
    ShuffleboardTab tab2 = Shuffleboard.getTab("Tab 2");
    int i;      
    String l_position;
    for(i = 1; i < 10; ++i) {
      l_position = String.format("ScorePos%d%s", i, scoringHeight.High.toString());
      tab2.add(l_position, false).withPosition(i - 1, 0);
      l_position = String.format("ScorePos%d%s", i, scoringHeight.Med.toString());
      tab2.add(l_position, false).withPosition(i - 1, 1);
      l_position = String.format("ScorePos%d%s", i, scoringHeight.Low.toString());
      tab2.add(l_position, false).withPosition(i - 1, 2);
    }

  }
  private void configureBindings() {
    
    driverJoystick.rightBumper().debounce(0.1).onTrue(new ToggleVisionState(vision));
    driverJoystick.b().debounce(0.1).whileTrue(new RunBottomPickup(pickup));
    driverJoystick.x().debounce(0.1).onTrue(new ToggleClaw(arms));
    driverJoystick.povUp().whileTrue(new MoveArmOut(arms));
    driverJoystick.povDown().whileTrue(new MoveArmIn(arms));
    driverJoystick.a().debounce(.1).whileTrue(new FingersIn(fingers));
    driverJoystick.y().debounce(.1).whileTrue(new FingersOut(fingers));

    //driverJoystick.axisGreaterThan(4, .5).onTrue(new ChangeScoringSlot(arms, true));
    //driverJoystick.axisLessThan(4, -.5).onTrue(new ChangeScoringSlot(arms, false));

    
    //driverJoystick.axisGreaterThan(5, .5).onTrue(new ChangeScoringHeight(arms, false));
    //driverJoystick.axisLessThan(5, -.5).onTrue(new ChangeScoringHeight(arms, true));

    //driverJoystick.povRight().onTrue(new RotateArmToPosition(arms, ArmRotate.maxPosition));
    //driverJoystick.povLeft().onTrue(new RotateArmToPosition(arms, ArmRotate.minPosition));
    driverJoystick.povRight().whileTrue(new RotateArm(arms, .25));
    driverJoystick.povLeft().whileTrue(new RotateArm(arms, -.25));
    configureButtonBoard();
  }

  private CommandJoystick buttonBoard = new CommandJoystick(1);
  private void configureButtonBoard(){
    buttonBoard.button(10).whileTrue(new RotateArm(arms, RobotPrefs.getArmRotateUpSpeed()));
    buttonBoard.button(11).whileTrue(new RotateArm(arms, RobotPrefs.getArmRotateDownSpeed()));

    buttonBoard.button(9).whileTrue(new MoveArmOut(arms));
    buttonBoard.button(3).whileTrue(new MoveArmIn(arms));

    buttonBoard.button(4).onTrue(new ToggleClaw(arms));
    
    buttonBoard.button(2).whileTrue(new RunBottomPickup(pickup));
    buttonBoard.button(8).whileTrue(new RunBottomPickup(pickup).alongWith(new FingersIn(fingers)));

    buttonBoard.button(6).onTrue(new CycleGrabPosition(arms));

    buttonBoard.axisGreaterThan(1, .5).onTrue(new ChangeScoringHeight(arms, false));
    buttonBoard.axisLessThan(1, -.5).onTrue(new ChangeScoringHeight(arms, true));
    
    buttonBoard.axisGreaterThan(0, .5).onTrue(new ChangeScoringSlot(arms, true));
    buttonBoard.axisLessThan(0, -.5).onTrue(new ChangeScoringSlot(arms, false));

    buttonBoard.button(5).onTrue(new MoveArmToPosition(arms, 0).andThen(new RotateArmToPosition(arms, 0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
