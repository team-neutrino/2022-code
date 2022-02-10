package frc.robot.commands.Autonomi.TwoBall;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.TrajectoryConfigs;

public class TwoBallTrajectory
{

    public static final Trajectory twoBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(1, 0, Rotation2d.fromDegrees(0))),
        TrajectoryConfigs.m_ForwardConfig);

}