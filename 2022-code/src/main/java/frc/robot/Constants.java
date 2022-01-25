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

    public static final class TurretConstants {
        public static final double TURRET_KP = 0.03;
        public static final double TURRET_KD = 0.02;
        public static final double TURRET_KI = 0.01;
        public static final double TURRET_UPDATE_ANGLE = 10;
        public static final double TURRET_DEAD_ANGLE = 1;
        public static final double TURRET_LIMIT_ANGLE = 160;
        public static final double TURRET_KP2 = 0.01;
    }

    public static final class PortConstants {
        public static final int XBOX_CONTROLLER_ID = 2;
    }

    public  final class JoystickConstants {
        public static final int LEFT_JOYSTICK_ID = 0;
        public static final int RIGHT_JOYSTICK_ID = 1;
    }

    public final class CANIDConstants {
        public static final int TURRET_MOTOR_ID = 7;
        public static final int DRIVETRAIN_MOTOR_RIGHT_1_ID = 1;
        public static final int DRIVETRAIN_MOTOR_RIGHT_2_ID = 2;
        public static final int DRIVETRAIN_MOTOR_LEFT_1_ID = 4;
        public static final int DRIVETRAIN_MOTOR_LEFT_2_ID = 5;
        public static final int SHOOTER_MOTOR_1_ID = 14;
        public static final int SHOOTER_MOTOR_2_ID =12;
        public static final int SHOOTER_MOTOR_3_ID = 11;
        public static final int INDEX_MOTOR_1_ID = 15;
        public static final int INDEX_MOTOR_2_ID = 16;
        }

        public static final class Shooter
        {
            public static final double WHEEL_P = 0.04;
            public static final double WHEEL_I = 0;
            public static final double WHEEL_D = 2;
            public static final double WHEEL_F = 0.008;
            public static final double SHOOTER_CONVERSION = 0;
        }
        public static final class IndexConstants
        {
            public static final int INDEX_BEAMBREAK = 7;
        }

       
    }
