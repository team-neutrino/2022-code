package frc.robot.commands.Trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.TrajectoryConfigConstants;
import java.util.List;

public class FourBallTrajectory {

  public static final Trajectory fourBall0 =
      TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(1.5, 0, Rotation2d.fromDegrees(0))),
          TrajectoryConfigConstants.m_diffForward);

  public static final Trajectory fourBall1 =
      TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(3.2, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(5.5, -.6, Rotation2d.fromDegrees(0))),
          TrajectoryConfigConstants.m_speedyForwardConfig);

  public static final Trajectory fourBall2 =
      TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(5.6, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
          TrajectoryConfigConstants.m_ReverseConfig);

  public static final Trajectory fourBall3 =
      TrajectoryGenerator.generateTrajectory(
          List.of(
              new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
              new Pose2d(1.5, 0, Rotation2d.fromDegrees(0))),
          TrajectoryConfigConstants.m_ReverseConfig);
}
