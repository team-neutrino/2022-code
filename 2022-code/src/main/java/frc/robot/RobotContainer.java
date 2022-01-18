// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.Controllers;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants.Controllers;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private XboxController m_OperatorController = new XboxController(Controllers.XBOX_CONTROLLER_PORT);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Joystick p_rightJoystick = new Joystick(Controllers.RIGHT_JOYSTICK_PORT);
  private Joystick p_leftJoystick = new Joystick(Controllers.LEFT_JOYSTICK_PORT);
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(p_rightJoystick,p_leftJoystick);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
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
  private void configureButtonBindings() 
  {
    m_B.whenHeld(new ShooterSetSpeed(m_Shooter, 50000));
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
