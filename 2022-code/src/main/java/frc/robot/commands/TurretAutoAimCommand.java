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

  private boolean m_hitLimit;
  private boolean m_notAuton;
  private double LIMELIGHT_MULTIPLICATION = 15.0;
  private double VELOCITY_DEADZONE = 0.1;
  private double ANGLE_MULTIPLIER = 50;
  /** Creates a new TurretAutoAimCommand. */
  public TurretAutoAimCommand(
      TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight, DriveTrainSubsystem p_drive, boolean p_notAuton) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    m_hitLimit = false;
    m_notAuton = p_notAuton;
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
    if (m_limelight.getTv() == true) {
      if (m_drive.getDriveEncoderL1() > VELOCITY_DEADZONE
          || m_drive.getDriveEncoderR2() > VELOCITY_DEADZONE) {
        if (m_drive.getNavYaw() < 0) {
          m_turret.setTargetAngle(
              m_turret.getCurrentAngle()
                  + LIMELIGHT_MULTIPLICATION * m_limelight.getTx()
                  - ANGLE_MULTIPLIER
                      * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1())
                      / 2);
        } else {
          m_turret.setTargetAngle(
              m_turret.getCurrentAngle()
                  + LIMELIGHT_MULTIPLICATION * m_limelight.getTx()
                  + ANGLE_MULTIPLIER
                      * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1())
                      / 2);
        }
        // uses limelight instead of NavX
        // if (m_limelight.getTx() > 0) {
        //   m_turret.setTargetAngle(
        //     m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx()
        //     - ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) /
        // 2);
        // }
        // else {
        //   m_turret.setTargetAngle(
        //     m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx()
        //     + ANGLE_MULTIPLIER * (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) /
        // 2);
        // }
      } else {
        m_turret.setTargetAngle(
            m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx());
      }
    // } else {
    //   if (m_notAuton) {
    //     if (m_turret.getCurrentAngle() <= m_turret.REVERSE_SOFT_LIMIT_THRESHOLD) {
    //       m_turret.setPower(.2);
    //       m_hitLimit = false;
    //     } else if (m_turret.getCurrentAngle() >= m_turret.FORWARD_SOFT_LIMIT_THRESHOLD - 40) {
    //       m_turret.setPower(-.2);
    //       m_hitLimit = true;
    //     } else {
    //       if (m_hitLimit) m_turret.setPower(-.2);
    //       else m_turret.setPower(.2);
    //     }
    //   }
    // }
  }
}

  public void setNotAuton() {
    m_notAuton = true;
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
