package frc.robot.commands.Trajectories;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.commands.AAAutonShootCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuton extends SequentialCommandGroup {
  /** Creates a new TenBallAuton. */
  private Trajectory m_fourBall0;

  private Trajectory m_fourBall1;
  private Trajectory m_fourBall2;

  public FourBallAuton(
      DriveTrainSubsystem p_drive,
      TurretPIDSubsystem p_turret,
      IntakeSubSystem p_intake,
      IndexSubsystem p_index,
      ShooterSubsystem p_shooter,
      LimelightSubsystem p_limelight) {
    m_fourBall0 = FourBallTrajectory.fourBall0;
    m_fourBall1 = FourBallTrajectory.fourBall1;
    m_fourBall2 = FourBallTrajectory.fourBall2;

    RamseteCommand fourBall0Command =
        new RamseteCommand(
            m_fourBall0,
            p_drive::getPose,
            new RamseteController(
                TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                TrajectoryConfigConstants.KS_VOLTS,
                TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER,
                TrajectoryConfigConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
            p_drive::getWheelSpeeds,
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            p_drive::setTankDriveVolts,
            p_drive);

    RamseteCommand fourBall1Command =
        new RamseteCommand(
            m_fourBall1,
            p_drive::getPose,
            new RamseteController(
                TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                TrajectoryConfigConstants.KS_VOLTS,
                TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER,
                TrajectoryConfigConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
            p_drive::getWheelSpeeds,
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            p_drive::setTankDriveVolts,
            p_drive);

    RamseteCommand fourBall2Command =
        new RamseteCommand(
            m_fourBall2,
            p_drive::getPose,
            new RamseteController(
                TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                TrajectoryConfigConstants.KS_VOLTS,
                TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER,
                TrajectoryConfigConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
            p_drive::getWheelSpeeds,
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
            p_drive::setTankDriveVolts,
            p_drive);

    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(fourBall0Command, new AutonIntakeCommand(p_intake, 2)),
            new InstantCommand(() -> p_drive.setTankDriveVolts(0.0, 0.0)),
            new AAAutonShootCommand(p_shooter, p_index, p_turret, p_limelight, 3),
            new InstantCommand(p_limelight::setLimelightOff),
            new ParallelCommandGroup(fourBall1Command, new AutonIntakeCommand(p_intake, 3)), 
            new InstantCommand(p_limelight::setLimelightOff),
            fourBall2Command,
            new InstantCommand(() -> p_drive.setTankDriveVolts(0.0, 0.0)),
            new AAAutonShootCommand(p_shooter, p_index, p_turret, p_limelight, 3)
                .alongWith(new AutonIntakeCommand(p_intake, 2))));
  }
}
