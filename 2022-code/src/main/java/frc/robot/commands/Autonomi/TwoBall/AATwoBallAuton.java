package frc.robot.commands.Autonomi.TwoBall;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryConfigConstants;
// import frc.robot.commands.AAAutonShootCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AATwoBallAuton extends SequentialCommandGroup {
  /** Creates a new TenBallAuton. */
  private Trajectory m_twoBall0;

  public AATwoBallAuton(
      DriveTrainSubsystem p_drive,
      TurretPIDSubsystem p_turret,
      IntakeSubSystem p_intake,
      IndexSubsystem p_index,
      ShooterSubsystem p_shooter,
      LimelightSubsystem p_limelight) {
    m_twoBall0 = TwoBallTrajectory.twoBall0;

    RamseteCommand twoBall0Command =
        new RamseteCommand(
            m_twoBall0,
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
        // new TurretAutoAimCommand(p_turret, p_limelight)
        //     .alongWith(new AAAutonShootCommand(p_shooter, p_index, 4.5)),
        // twoBall0Command.alongWith(new IntakeCommand(p_intake)),
        // new TurretAutoAimCommand(p_turret, p_limelight)
        //     .alongWith(new AAAutonShootCommand(p_shooter, p_index, 4.5)));
        );
  }
}
