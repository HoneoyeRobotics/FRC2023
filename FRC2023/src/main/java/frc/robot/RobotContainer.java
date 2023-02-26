// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.enums.*;

public class RobotContainer {

  public ScoringHeight scoringHeight;
  public DriveTrain drivetrain;
  public Vision vision;
  public Pickup pickup;
  public Arms arms;
  public Fingers fingers;
  private CommandXboxController driverJoystick = new CommandXboxController(0);
  private CommandJoystick buttonBoard = new CommandJoystick(1);

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
    configureButtonBoard();

    SmartDashboard.putData(new ResetArmLengthEncoder(arms));
    SmartDashboard.putData(new ResetArmRotateEncoder(arms));
    SmartDashboard.putData(new ToggleArmLengthBrake(arms));
    SmartDashboard.putData(new RotateToPeg(vision, drivetrain));
    SmartDashboard.putData(new BalanceOnPlatform(drivetrain, false));
    SmartDashboard.putData(new ToggleClaw(arms));
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
  
  private void configureBindings() {
    
    driverJoystick.rightBumper().whileTrue(new BrakeRobot(drivetrain));
    
    driverJoystick.x().debounce(0.1).onTrue(new ToggleClaw(arms));
    driverJoystick.povUp().whileTrue(new MoveArmOut(arms));
    driverJoystick.povDown().whileTrue(new MoveArmIn(arms));
    driverJoystick.povLeft().whileTrue(new RunBottomPickup(pickup));
    driverJoystick.povRight().whileTrue(new ReverseBottomPickup(pickup));    
    driverJoystick.a().debounce(.1).whileTrue(new FingersIn(fingers));
    driverJoystick.y().debounce(.1).whileTrue(new FingersIn(fingers));
    driverJoystick.povRight().whileTrue(new RotateArm(arms, true));
    driverJoystick.povLeft().whileTrue(new RotateArm(arms, false));
    driverJoystick.back().whileTrue(new BalanceOnPlatform(drivetrain, false));
    driverJoystick.start().onTrue(new ToggleArmRotatePID(arms));
  }

  private void configureButtonBoard(){
    //buttonBoard.button(2).whileTrue(new RunBottomPickup(pickup));
    buttonBoard.button(8).whileTrue(new RunBottomPickup(pickup).alongWith(new FingersIn(fingers)));

    buttonBoard.button(6).onTrue(new CycleGrabPosition(arms));
    buttonBoard.button(7).onTrue(new  GrabPiece(arms));

    buttonBoard.button(4).onTrue(new ToggleClaw(arms));

    buttonBoard.button(9).whileTrue(new MoveArmOut(arms));
    buttonBoard.button(3).whileTrue(new MoveArmIn(arms));

    buttonBoard.button(10).whileTrue(new RotateArm(arms, true));
    buttonBoard.button(11).whileTrue(new RotateArm(arms, false));

    //Brings the arm home
    buttonBoard.button(5).onTrue(new CloseClaw(arms)
      .andThen(new MoveArmCompletelyIn(arms))
      .andThen(new RotateArmToPositionPID(arms, 0))
      .andThen(new OpenClaw(arms)));

    //Changes the scoring position grid on dashboard
    buttonBoard.axisGreaterThan(1, .5).onTrue(new ChangeScoringHeight(arms, true));
    buttonBoard.axisLessThan(1, -.5).onTrue(new ChangeScoringHeight(arms, false));

    buttonBoard.axisGreaterThan(0, .5).onTrue(new ChangeScoringSlot(arms, true));
    buttonBoard.axisLessThan(0, -.5).onTrue(new ChangeScoringSlot(arms, false));

    buttonBoard.button(2).onTrue(new ScorePiece(arms));
  }

  public Command getAutonomousCommand() {
    //Autonomous 1 should back up turn 180 and balance on the platfrom
    //TODO: add Autonomous 2 that should score first then do the above
    return new Autonomous1(arms, drivetrain, scoringHeight);
    //Commands.print("No autonomous command configured");
  }
}
