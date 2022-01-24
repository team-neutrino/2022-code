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
public final class Constants 
{

    public static final class ShuffleboardConstants 
    {
        public static final String THEME_SRCURL = "src/main/java/frc/robot/util/themes/";
    }

    public static final class CanId
    {
        public static final int MOTOR_CONTROLLER_INTAKE_FEED = 15;
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT1 = 1;
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT2 = 2;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT1 = 4;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT2 = 5;
        public static final int MOTOR_CONTROLLER_SHOOTER1 = 14;
        public static final int MOTOR_CONTROLLER_SHOOTER2 = 12;
        public static final int MOTOR_CONTROLLER_SHOOTER3 = 11;
    }
    public static final class SolenoidId
    {
        public static final int SOLENOID_INTAKE_FORWARD = 0;
        public static final int SOLENOID_INTAKE_REVERSE = 0;

    }
    public static final class IntakeConstants{
        public static final double INTAKE_MOTOR_POWER = -1;
        public static final double OUTTAKE_MOTOR_POWER = 1;
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
        public static final double WHEEL_F = 0.008;
        public static final double SHOOTER_CONVERSION = 0;
    }
}
