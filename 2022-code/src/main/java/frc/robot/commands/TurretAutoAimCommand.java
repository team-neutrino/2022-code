// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class TurretAutoAimCommand extends CommandBase {
  private TurretPIDSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  private DriveTrainSubsystem m_drive;

  private double LIMELIGHT_MULTIPLICATION = 10.0;
  private double VELOCITY_DEADZONE = 0.1;
  private double ANGLE_MULTIPLIER = 10;
  /** Creates a new TurretAutoAimCommand. */
  public TurretAutoAimCommand(TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight, DriveTrainSubsystem p_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    m_drive = p_drive;
    addRequirements(m_turret, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLimelightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setLimelightOn();
    if (m_limelight.getTv() == true) {
      if (m_drive.getDriveEncoderL1() > VELOCITY_DEADZONE || m_drive.getDriveEncoderR2() > VELOCITY_DEADZONE) {
          if (m_drive.getNavYaw() > 0) {
            m_turret.setTargetAngle(
              m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx() 
              - ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) / 2);
          }
          else {
            m_turret.setTargetAngle(
              m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx() 
              + ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) / 2);
          }
          // uses limelight instead of NavX
          // if (m_limelight.getTx() > 0) {
          //   m_turret.setTargetAngle(
          //     m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx() 
          //     - ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) / 2);
          // }
          // else {
          //   m_turret.setTargetAngle(
          //     m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx() 
          //     + ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) / 2);
          // }
      }
      else {
        m_turret.setTargetAngle(
            m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx());
      }
    } else {
      m_turret.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
