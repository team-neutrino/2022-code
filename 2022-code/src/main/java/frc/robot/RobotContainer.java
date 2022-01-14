// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Joystick m_rightJoystick = new Joystick(0);
  private final Joystick m_leftJoystick = new Joystick(1);
  private double rightY;
  private double leftY;
  private final TalonSRX m_leftMotor0 = new TalonSRX(9);
  private final TalonSRX m_leftMotor1 = new TalonSRX(10);
  private final TalonSRX m_rightMotor0 = new TalonSRX(13);
  private final TalonSRX m_rightMotor1 = new TalonSRX(14);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_rightMotor0.setInverted(true);
    m_rightMotor1.setInverted(true);
    m_leftMotor0.setInverted(true);
    m_leftMotor1.setInverted(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  public void driveupdate() {
    leftY = m_leftJoystick.getY();
    rightY = m_rightJoystick.getY();
    m_rightMotor0.set(ControlMode.PercentOutput, rightY);
    m_rightMotor1.set(ControlMode.PercentOutput, rightY);
   m_leftMotor0.set(ControlMode.PercentOutput, leftY*-1);
    m_leftMotor1.set(ControlMode.PercentOutput, leftY*-1);
  }
}
