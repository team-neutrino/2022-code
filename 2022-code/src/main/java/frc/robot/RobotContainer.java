// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.TurretManualAimCommand;
import frc.robot.commands.TurretToAngleCommand;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** Instantiate buttons, joysticks, etc. below */
  private XboxController m_OperatorController = new XboxController(Constants.PortConstants.XBOX_CONTROLLER_ID);
  private POVButton m_leftPovButton = new POVButton(m_OperatorController, 270);
  private POVButton m_rightPovButton = new POVButton(m_OperatorController, 90);
  private Joystick m_rightJoystick = new Joystick(Constants.JoystickConstants.RIGHT_JOYSTICK_ID);
  private Joystick m_leftJoystick = new Joystick(Constants.JoystickConstants.LEFT_JOYSTICK_ID);
  private JoystickButton m_A = new JoystickButton(m_OperatorController, XboxController.Button.kA.value);

  /** Instantiate subsystems below */
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private TurretSubsystem m_turret = new TurretSubsystem();
  private DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(m_rightJoystick, m_leftJoystick);
  private ShuffleboardSubsystem m_shuffleboard = new ShuffleboardSubsystem(m_turret);
  private limelightSubsystem m_limelight = new limelightSubsystem();

  /** Instantiate commands below */
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_turret.setDefaultCommand(new TurretAutoAimCommand(m_turret, m_limelight));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
  
    /** Turret mappings */
    m_leftPovButton.whileHeld(new TurretManualAimCommand(m_turret, true)).whenReleased(new InstantCommand(m_turret::stop));
    m_rightPovButton.whileHeld(new TurretManualAimCommand(m_turret, false)).whenReleased(new InstantCommand(m_turret::stop));
    m_A.whenPressed(new TurretToAngleCommand(m_turret, 150));

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
