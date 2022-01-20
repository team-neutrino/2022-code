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
    public static final class  CanId
    {
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT1 = 1;
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT2 = 2;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT1 = 4;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT2 = 5;
        public static final int MOTOR_CONTROLLER_SHOOTER1 = 1;
        public static final int MOTOR_CONTROLLER_SHOOTER2 =2;
        public static final int MOTOR_CONTROLLER_SHOOTER3 = 11;
    }
    public static final class Controllers
    {
        public static final int XBOX_CONTROLLER_PORT = 2;
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
    }
    public static final class Shooter
    {
        public static final double WHEEL_P = 0.04;
        public static final double WHEEL_I = 0;
        public static final double WHEEL_D = 2;
        public static final double SHOOTER_CONVERSION = 0;
        public static final double SHOOTER_SPEED = 50000;
    }
}
