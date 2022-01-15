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
        public static final double TURRET_KP = 0.1;
        public static final double TURRET_UPDATE_ANGLE = 2;
        public static final double TURRET_LIMIT_ANGLE = 135;
    }

    public static final class PortConstants {
        public static final int XBOX_CONTROLLER_ID = 0;
    }

    public  final class JoystickCON {
        public static final int LEFT_JOYSTICK_IMPUT = 0;
        public static final int RIGHT_JOYSTICK_INPUT = 1;

    }
    public final class MotorCON{

        public static final int TURRET_MOTOR_ID = 0;
        public static final int DRIVETRAIN_MOTOR_ID_RIGHT_1 = 1;
        public static final int DRIVETRAIN_MOTOR_ID_RIGHT_2 = 2;
        public static final int DRIVETRAIN_MOTOR_ID_LEFT_1 = 3;
        public static final int DRIVETRAIN_MOTOR_ID_LEFT_2 = 4;



    }
}
