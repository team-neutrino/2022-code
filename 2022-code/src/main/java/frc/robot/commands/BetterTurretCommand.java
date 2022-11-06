// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class BetterTurretCommand extends CommandBase {
  private TurretPIDSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  private DriveTrainSubsystem m_drive;
  private boolean m_notAuton;
  private double LIMELIGHT_MULTIPLICATION = 10.0;

  /** Creates a new TurretAutoAimCommand. */
  public BetterTurretCommand(
      TurretPIDSubsystem p_turret,
      DriveTrainSubsystem p_drive,
      LimelightSubsystem p_limelight,
      boolean p_notAuton) {
    m_turret = p_turret;
    m_limelight = p_limelight;
    m_drive = p_drive;
    m_notAuton = p_notAuton;
    addRequirements(m_turret, m_limelight);
  }

  @Override
  public void initialize() {
    m_limelight.setLimelightOn();
  }

  // LIMELIGHT_MULTIPLICATION * m_limelight.getTx() + (I don't think this is needed. Try it with and without)

  @Override
  public void execute() {
    if (m_limelight.getTv() == true) {
      m_turret.setTargetAngle(
          m_turret.getCurrentAngle() + m_turret.movePositionFeedfoward());
      m_drive.resetNavX();
    } /*else {
      if (m_notAuton) {
        m_turret.setTargetAngle(
            m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_drive.getNavYaw());
      }
    }
    */
  }

  public void setNotAuton() {
    m_notAuton = true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
