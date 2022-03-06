// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Autonomi.TwoBall.TwoBallAuton;
import frc.robot.commands.Autonomi.TwoBall.TwoBallTrajectory;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbKeyExtendCommand;
import frc.robot.commands.ClimbKeyUnlockCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.IndexDefaultCommand;
import frc.robot.commands.IndexManualCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterInterpolateSpeed;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.TestShooterRPMCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.TurretManualAimCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.util.AutonSelector;
import frc.robot.util.TriggerToBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Instantiate buttons, joysticks, etc. below */
  private XboxController m_OperatorController =
      new XboxController(Constants.ControllerConstants.XBOX_CONTROLLER_ID);

  private POVButton m_leftPovButton = new POVButton(m_OperatorController, 270);
  private POVButton m_upPovButton = new POVButton(m_OperatorController, 0);
  private POVButton m_downPovButton = new POVButton(m_OperatorController, 180);
  private POVButton m_rightPovButton = new POVButton(m_OperatorController, 90);
  private Joystick m_rightJoystick = new Joystick(Constants.ControllerConstants.RIGHT_JOYSTICK_ID);
  private Joystick m_leftJoystick = new Joystick(Constants.ControllerConstants.LEFT_JOYSTICK_ID);
  private JoystickButton m_BumperLeft =
      new JoystickButton(m_OperatorController, XboxController.Button.kLeftBumper.value);
  private JoystickButton m_BumperRight =
      new JoystickButton(m_OperatorController, XboxController.Button.kRightBumper.value);
  private JoystickButton m_B =
      new JoystickButton(m_OperatorController, XboxController.Button.kB.value);
  private JoystickButton m_A =
      new JoystickButton(m_OperatorController, XboxController.Button.kA.value);
  private JoystickButton m_Y =
      new JoystickButton(m_OperatorController, XboxController.Button.kY.value);
  private JoystickButton m_X =
      new JoystickButton(m_OperatorController, XboxController.Button.kX.value);
  private JoystickButton m_start =
      new JoystickButton(m_OperatorController, XboxController.Button.kStart.value);
  private JoystickButton m_back =
      new JoystickButton(m_OperatorController, XboxController.Button.kBack.value);
  private JoystickButton m_rightJoystickButton =
      new JoystickButton(m_OperatorController, XboxController.Button.kRightStick.value);
  private TriggerToBoolean m_TriggerLeft =
      new TriggerToBoolean(m_OperatorController, Axis.kLeftTrigger.value);
  private TriggerToBoolean m_TriggerRight =
      new TriggerToBoolean(m_OperatorController, Axis.kRightTrigger.value);

  private TwoBallTrajectory twoBallTrajectory = new TwoBallTrajectory();

  /** Instantiate subsystems below */
  private final IndexSubsystem m_index = new IndexSubsystem();

  private final TurretPIDSubsystem m_turret = new TurretPIDSubsystem();
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem();
  private final IntakeSubSystem m_intake = new IntakeSubSystem();
  private final Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_limelight);
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ShuffleboardSubsystem m_shuffleboard =
      new ShuffleboardSubsystem(m_shooter, m_turret, m_climber, m_driveTrain, m_index, m_limelight);

  /** Instantiate default command below */
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intake);

  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
      new DriveTrainDefaultCommand(m_driveTrain, m_leftJoystick, m_rightJoystick);
  private final TurretAutoAimCommand m_turretAutoAimCommand =
      new TurretAutoAimCommand(m_turret, m_limelight);
  private final ShooterDefaultCommand m_shooterDefaultCommand =
      new ShooterDefaultCommand(m_shooter);

  private final TwoBallAuton m_twoBallAuton =
      new TwoBallAuton(m_driveTrain, m_turret, m_intake, m_shooter);
  private final AutonSelector m_autonSelector =
      new AutonSelector(m_driveTrain, m_turret, m_intake, m_shooter, m_limelight);

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
    m_index.setDefaultCommand(new IndexDefaultCommand(m_index));
    m_intake.setDefaultCommand(m_intakeDefaultCommand);
    m_turret.setDefaultCommand(m_turretAutoAimCommand);
    m_shooter.setDefaultCommand(m_shooterDefaultCommand);
    m_climber.setDefaultCommand(new ClimbDefaultCommand(m_climber));

    /** xbox button mapping */
    m_B.whileHeld(new IndexManualCommand(m_index));
    m_X.whileHeld(new TestShooterRPMCommand(m_shooter));
    m_downPovButton.whileHeld(
        new SequentialCommandGroup(
            new ClimbKeyUnlockCommand(m_climber),
            new WaitCommand(0.5),
            new ClimbRetractCommand(m_climber)));
    m_upPovButton.whileHeld(
        new SequentialCommandGroup(
            new ClimbKeyUnlockCommand(m_climber),
            new WaitCommand(0.5),
            new ClimbExtendCommand(m_climber)));
    m_back.whenReleased(new ClimbKeyExtendCommand(m_climber));
    m_BumperLeft.whileActiveContinuous(new ShooterSetSpeed(m_shooter, 750));
    m_TriggerRight.whileActiveContinuous(new ShooterInterpolateSpeed(m_shooter));
    m_TriggerLeft.whileActiveContinuous(new IntakeCommand(m_intake));
    m_leftPovButton.whileHeld(new TurretManualAimCommand(m_turret, false));
    m_rightPovButton.whileHeld(new TurretManualAimCommand(m_turret, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    m_driveTrain.resetOdometry(m_driveTrain.getPose());
    return m_twoBallAuton;
  }
}
