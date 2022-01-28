// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.TestShooterRPMCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.TurretManualAimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TriggerToBoolean;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.commands.TurretManualAimCommand;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveTrainSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Instantiate buttons, joysticks, etc. below */
  private XboxController m_OperatorController = new XboxController(Constants.ControllerConstants.XBOX_CONTROLLER_ID);
  private POVButton m_leftPovButton = new POVButton(m_OperatorController, 270);
  private POVButton m_rightPovButton = new POVButton(m_OperatorController, 90);
  private Joystick m_rightJoystick = new Joystick(Constants.ControllerConstants.RIGHT_JOYSTICK_ID);
  private Joystick m_leftJoystick = new Joystick(Constants.ControllerConstants.LEFT_JOYSTICK_ID);
  private JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
  private JoystickButton m_A = new JoystickButton(m_OperatorController, XboxController.Button.kA.value);
  private JoystickButton m_start = new JoystickButton(m_OperatorController, XboxController.Button.kStart.value);

  /** Instantiate subsystems below */
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final IntakeSubSystem m_intake = new IntakeSubSystem();
  private Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final ShuffleboardSubsystem m_shuffleboard = new ShuffleboardSubsystem(m_shooter);

   /** Instantiate command below */
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intake);
  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand = new DriveTrainDefaultCommand(m_driveTrain, m_rightJoystick,m_leftJoystick);
  private final ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand(m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_compressor.enableDigital();
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
    /** default command mapping */
    m_driveTrain.setDefaultCommand(m_driveTrainDefaultCommand);
    m_intake.setDefaultCommand(m_intakeDefaultCommand);
    m_shooter.setDefaultCommand(m_shooterDefaultCommand);

    /** xbox button mapping */
    m_A.whileHeld(new IntakeCommand(m_intake));
    m_B.whileHeld(new ShooterSetSpeed(m_shooter, m_shooter.getTargetRPM()));
    m_start.whileHeld(new TestShooterRPMCommand(m_shooter, m_shuffleboard));
    m_leftPovButton.whileHeld(new TurretManualAimCommand(m_turret, true));
    m_rightPovButton.whileHeld(new TurretManualAimCommand(m_turret, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_intakeDefaultCommand;
  }
}