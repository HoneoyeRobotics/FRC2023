// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveRobotArcade;
import frc.robot.commands.ToggleTurret;
import frc.robot.commands.FollowLimeLight;
import frc.robot.commands.RunTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Turret turret = new Turret();
  Joystick driverJoystick = new Joystick(0);
  
  
  private final DriveTrain driveTrain = new DriveTrain();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveTrain.setDefaultCommand(new DriveRobotArcade(driveTrain,
    () -> driverJoystick.getRawAxis(Constants.AXIS_RightTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftStickX)));


    JoystickButton buttonA = new JoystickButton(driverJoystick, 1);
    buttonA.whenPressed(new ToggleTurret(turret));
  

    JoystickButton buttonB = new JoystickButton(driverJoystick, 2);
    buttonB.whileHeld(new RunTurret(turret, () -> driverJoystick.getRawAxis(Constants.AXIS_RightStickX)));


    
    JoystickButton buttony = new JoystickButton(driverJoystick, 3);
    buttony.whileHeld(new FollowLimeLight(driveTrain));
  

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
