package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class TrajectoryConfigConstants {
  public static final double KV_VOLT_SECONDS_PER_METER = 2.4661;
  public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.5174;
  public static final double K_TRACK_WIDTH_METERS = 0.668;
  public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS =
      new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
  public static final double K_MAX_SPEED_METERS_PER_SECOND = 1;
  public static final double K_HALF_SPEED_METERS_PER_SECOND = 0.5;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.5;
  public static final double K_RAMSETE_BETA = 2;
  public static final double K_RAMSETE_ZETA = 0.7;
  public static final double KP_DRIVE_VEL = 3.4239;
  public static final double KS_VOLTS = 0.19143;

  private static final DifferentialDriveVoltageConstraint m_autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
          K_DRIVE_KINEMATICS,
          10);

  public static final TrajectoryConfig m_ForwardConfig =
      new TrajectoryConfig(10, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);

  public static final TrajectoryConfig m_diffForward =
      new TrajectoryConfig(15, 2)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);

  public static final TrajectoryConfig m_speedyForwardConfig =
      new TrajectoryConfig(14, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);

  public static final TrajectoryConfig m_ReverseConfig =
      new TrajectoryConfig(8, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint)
          .setReversed(true);
}
