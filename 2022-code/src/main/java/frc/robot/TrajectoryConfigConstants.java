package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class TrajectoryConfigConstants {
  public static final double KV_VOLT_SECONDS_PER_METER = 3.21;
  public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.444;
  public static final double K_TRACK_WIDTH_METERS = 0.7;
  public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS =
      new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
  public static final double K_MAX_SPEED_METERS_PER_SECOND = 1;
  public static final double K_HALF_SPEED_METERS_PER_SECOND = .5;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 6.8;
  public static final double K_RAMSETE_BETA = 2;
  public static final double K_RAMSETE_ZETA = 0.7;
  public static final double KP_DRIVE_VEL = 2.15;
  public static final double KS_VOLTS = 0.177;

  private static final DifferentialDriveVoltageConstraint m_autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
          K_DRIVE_KINEMATICS,
          10);

  public static final TrajectoryConfig m_ForwardConfig =
      new TrajectoryConfig(
              K_HALF_SPEED_METERS_PER_SECOND, K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(K_DRIVE_KINEMATICS)
          .addConstraint(m_autoVoltageConstraint);
}

