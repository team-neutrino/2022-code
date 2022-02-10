

package frc.robot.commands.Autonomi.TwoBall;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.TrajectoryConfigs;
import frc.robot.commands.IntakeCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup
{
    /** Creates a new TenBallAuton. */
    private Trajectory m_twoBall0;

    public TwoBallAuton(DriveTrainSubsystem p_drive, 
                        TurretPIDSubsystem p_turret, 
                        IntakeSubsystem p_intake, 
                        ShooterSubsystem p_shooter)
    {

        m_twoBall0 = TwoBallTrajectory.twoBall0;

        RamseteCommand twoBall0Command = 
        new RamseteCommand(
            m_twoBall0, 
            p_drive::getPose,
            new RamseteController(TrajectoryConfigs.K_RAMSETE_BETA, TrajectoryConfigs.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(TrajectoryConfigs.KS_VOLTS,
                                       TrajectoryConfigs.KV_VOLT_SECONDS_PER_METER,
                                       TrajectoryConfigs.KA_VOLT_SECONDS_SQUARED_PER_METER),
            TrajectoryConfigs.K_DRIVE_KINEMATICS, 
            p_drive::getWheelSpeeds,
            new PIDController(TrajectoryConfigs.KP_DRIVE_VEL, 0, 0),
            new PIDController(TrajectoryConfigs.KP_DRIVE_VEL, 0, 0),
            p_drive::setTankDriveVolts,
            p_drive);

        addCommands(twoBall0Command, 
                    new IntakeCommand(p_intake, true), new InstantCommand(() -> p_drive.setTankDriveVolts(0, 0)));
    }
}