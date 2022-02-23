// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
import frc.robot.commands.IndexManualCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterInterpolateSpeed;
import frc.robot.commands.ShooterSetSpeed;
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
  private POVButton m_rightPovButton = new POVButton(m_OperatorController, 90);
  private Joystick m_rightJoystick = new Joystick(Constants.ControllerConstants.RIGHT_JOYSTICK_ID);
  private Joystick m_leftJoystick = new Joystick(Constants.ControllerConstants.LEFT_JOYSTICK_ID);
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
  private TriggerToBoolean m_TriggerLeft =
      new TriggerToBoolean(m_OperatorController, Axis.kLeftTrigger.value);

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

  private final TwoBallAuton m_twoBallAuton =
      new TwoBallAuton(m_driveTrain, m_turret, m_intake, m_shooter);
  /** Instantiate default command below */
  private final IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand(m_intake);

  private final DriveTrainDefaultCommand m_driveTrainDefaultCommand =
  new DriveTrainDefaultCommand(m_driveTrain, m_rightJoystick, m_leftJoystick);
  private final TurretAutoAimCommand m_turretAutoAimCommand =
  new TurretAutoAimCommand(m_turret, m_limelight);
  private final ShooterDefaultCommand m_shooterDefaultCommand =
  new ShooterDefaultCommand(m_shooter);

  private AutonSelector m_autonSelector =
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
    // m_index.setDefaultCommand(new IndexDefaultCommand(m_index));
    m_intake.setDefaultCommand(m_intakeDefaultCommand);
    m_turret.setDefaultCommand(m_turretAutoAimCommand);
    m_shooter.setDefaultCommand(m_shooterDefaultCommand);
    m_TriggerLeft.whileActiveOnce(new IntakeCommand(m_intake));
    m_climber.setDefaultCommand(new ClimbDefaultCommand(m_climber));

    /** xbox button mapping */
    m_Y.whileHeld(new IndexManualCommand(m_index));
    m_A.whileHeld(new IntakeCommand(m_intake));
    m_B.whileHeld(new ShooterSetSpeed(m_shooter));
    m_X.whileHeld(new ShooterInterpolateSpeed(m_shooter));
    m_start.whenHeld(
        new SequentialCommandGroup(
            new ClimbKeyUnlockCommand(m_climber),
            new WaitCommand(0.5),
            new ClimbExtendCommand(m_climber)));
    m_back.whenHeld(
        new SequentialCommandGroup(
            new ClimbKeyUnlockCommand(m_climber),
            new WaitCommand(0.5),
            new ClimbRetractCommand(m_climber),
            new ClimbKeyExtendCommand(m_climber)));
    m_back.whenReleased(new ClimbKeyExtendCommand(m_climber));
    m_leftPovButton.whileHeld(new TurretManualAimCommand(m_turret, false));
    m_rightPovButton.whileHeld(new TurretManualAimCommand(m_turret, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                TrajectoryConfigConstants.KS_VOLTS,
                TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER,
                TrajectoryConfigConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                TrajectoryConfigConstants.K_MAX_SPEED_METERS_PER_SECOND,
                TrajectoryConfigConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(TrajectoryConfigConstants.K_DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_driveTrain::getPose,
            new RamseteController(TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                TrajectoryConfigConstants.KS_VOLTS,
                TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER,
                TrajectoryConfigConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
            m_driveTrain::getWheelSpeeds,
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            m_driveTrain::setTankDriveVolts,
            m_driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.setTankDriveVolts(0, 0));
  }
}

