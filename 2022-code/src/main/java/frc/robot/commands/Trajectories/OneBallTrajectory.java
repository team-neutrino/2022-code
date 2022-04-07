package frc.robot.commands.Trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.TrajectoryConfigConstants;
import java.util.List;

public class OneBallTrajectory {

  public static final Trajectory oneBall0 =
      TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(1.5, 0, Rotation2d.fromDegrees(0))),
          TrajectoryConfigConstants.m_diffForward);
}
