package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainDefaultCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private Joystick m_rightJoystick;
  private Joystick m_leftJoystick;
  private Timer m_encoderTimer = new Timer();
  private Timer m_ampTimer = new Timer();

  public DriveTrainDefaultCommand(
      DriveTrainSubsystem subsystem, Joystick p_leftJoystick, Joystick p_rightJoystick) {
    m_driveTrainSubsystem = subsystem;
    m_leftJoystick = p_leftJoystick;
    m_rightJoystick = p_rightJoystick;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_encoderTimer.start();
    m_ampTimer.start();
  }

  @Override
  public void execute() {
    m_driveTrainSubsystem.setMotors(
        squareDrive(m_leftJoystick.getY()), squareDrive(m_rightJoystick.getY()));
    if (m_encoderTimer.get() >= 5) {
      if (m_driveTrainSubsystem.getDriveEncoderR1() == 0) {
        System.out.println("Right Encoder 1 Fail");
      }
      if (m_driveTrainSubsystem.getDriveEncoderR2() == 0) {
        System.out.println("Right Encoder 2 Fail");
      }
      if (m_driveTrainSubsystem.getDriveEncoderL1() == 0) {
        System.out.println("Left Encoder 1 Fail");
      }
      if (m_driveTrainSubsystem.getDriveEncoderL2() == 0) {
        System.out.println("Left Encoder 2 Fail");
      }
    }
    if (Math.abs(m_driveTrainSubsystem.getDriveEncoderR1()) > 1
        && Math.abs(m_driveTrainSubsystem.getDriveEncoderR2()) > 1
        && Math.abs(m_driveTrainSubsystem.getDriveEncoderL1()) > 1
        && Math.abs(m_driveTrainSubsystem.getDriveEncoderL2()) > 1) {
      m_encoderTimer.reset();
    }

    if (m_ampTimer.get() >= 5) {
      if (m_driveTrainSubsystem.getAmpsR1() == 0) {
        System.out.println("Right Motor 1 Fail");
      }
      if (m_driveTrainSubsystem.getAmpsR2() == 0) {
        System.out.println("Right Motor 2 Fail");
      }
      if (m_driveTrainSubsystem.getAmpsL1() == 0) {
        System.out.println("Left Motor 1 Fail");
      }
      if (m_driveTrainSubsystem.getAmpsL2() == 0) {
        System.out.println("Left Motor 2 Fail");
      }
    }
    if (Math.abs(m_driveTrainSubsystem.getAmpsR1()) > 1
        && Math.abs(m_driveTrainSubsystem.getAmpsR2()) > 1
        && Math.abs(m_driveTrainSubsystem.getAmpsL1()) > 1
        && Math.abs(m_driveTrainSubsystem.getAmpsL2()) > 1) {
      m_ampTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public double squareDrive(double joystickVal) {
    final double SCALE = 1.2;
    return Math.pow(joystickVal, 2) * ((joystickVal < 0) ? -SCALE : SCALE);
  }
}
