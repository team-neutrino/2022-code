// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controllers;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TriggerToBoolean;

import static edu.wpi.first.wpilibj.XboxController.Button;


import frc.robot.subsystems.DriveTrainSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private XboxController m_OperatorController = new XboxController(Controllers.XBOX_CONTROLLER_PORT);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Joystick m_rightJoystick = new Joystick(Constants.Controllers.RIGHT_JOYSTICK_PORT);
  private Joystick m_leftJoystick = new Joystick(Constants.Controllers.LEFT_JOYSTICK_PORT);
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private TriggerToBoolean m_TriggerLeft = new TriggerToBoolean(m_OperatorController, Axis.kLeftTrigger.value,
  Constants.SolenoidId.SOLENOID_INTAKE_FORWARD);
  private final IntakeSubSystem p_trigger =  new IntakeSubSystem();
  
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand = new DriveTrainDefaultCommand(m_driveTrain, m_rightJoystick,m_leftJoystick);
  
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
    m_TriggerLeft.whenActive(new IntakeCommand(p_trigger, true));

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
