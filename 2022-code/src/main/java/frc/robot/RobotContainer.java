// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TurretContinuousTurnCommand;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants.JoystickCON;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


 
  /**
   * Instantiate subsystems below
   */
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private TurretSubsystem m_turret = new TurretSubsystem();

  /**
    * Instantiate commands below
    */
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * Instantiate controller ports below
   */

  private XboxController m_OperatorController = new XboxController(Constants.PortConstants.XBOX_CONTROLLER_ID);
  private POVButton m_leftPovButton = new POVButton(m_OperatorController, 0);
  private POVButton m_rightPovButton = new POVButton(m_OperatorController, 180);


  private Joystick p_rightJoystick = new Joystick(JoystickCON.RIGHT_JOYSTICK_INPUT);
  private Joystick p_leftJoystick = new Joystick(JoystickCON.LEFT_JOYSTICK_IMPUT);
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(p_rightJoystick,p_leftJoystick);

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
    
  /**
   * turret 
   */
    m_leftPovButton.whileHeld(new TurretContinuousTurnCommand(true)).whenReleased(
      new InstantCommand(m_turret::stop, m_turret));
    m_rightPovButton.whileHeld(new TurretContinuousTurnCommand(false)).whenReleased(
      new InstantCommand(m_turret::stop, m_turret));

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
