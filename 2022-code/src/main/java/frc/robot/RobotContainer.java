// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrainSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Joystick m_rightJoystick = new Joystick(Constants.Controllers.RIGHT_JOYSTICK_PORT);
  private Joystick m_leftJoystick = new Joystick(Constants.Controllers.LEFT_JOYSTICK_PORT);
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem, m_rightJoystick,m_leftJoystick);
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand = new DriveTrainDefaultCommand(m_driveTrain, m_rightJoystick,m_leftJoystick);
  private limelightSubsystem m_limelight = new limelightSubsystem();
  
  private ShuffleboardSubsystem shuffleboard = new ShuffleboardSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveTrain.setDefaultCommand(m_driveTrainDefaultCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
