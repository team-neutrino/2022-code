// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // subsystem dogmatic naming convention: {subsystem}_{aspect}_{}

  public final class ControllerConstants {
    public static final int LEFT_JOYSTICK_ID = 1;
    public static final int RIGHT_JOYSTICK_ID = 0;
    public static final int XBOX_CONTROLLER_ID = 2;
  }

  public final class CANIDConstants {
    public static final int DRIVETRAIN_MOTOR_LEFT_1_ID = 1;
    public static final int DRIVETRAIN_MOTOR_LEFT_2_ID = 2;
    public static final int DRIVETRAIN_MOTOR_RIGHT_1_ID = 3;
    public static final int DRIVETRAIN_MOTOR_RIGHT_2_ID = 4;
    public static final int SHOOTER_MOTOR_1_ID = 5;
    public static final int SHOOTER_MOTOR_2_ID = 6;
    public static final int TURRET_MOTOR_ID = 7;
    public static final int CLIMBER_MOTOR_1 = 8;
    public static final int CLIMBER_MOTOR_2 = 9;
    public static final int MOTOR_CONTROLLER_INTAKE_FEED = 10;
    public static final int INDEX_MOTOR_1_ID = 11;
    public static final int INDEX_MOTOR_2_ID = 12;
  }

  public static final class DigitalConstants {
    public static final int INDEX_BEAMBREAK = 7;
    public static final int CLIMBER_SWITCH = 1;
  }
}
