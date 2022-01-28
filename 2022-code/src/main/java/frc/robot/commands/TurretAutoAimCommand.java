// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class TurretAutoAimCommand extends CommandBase {
  private TurretPIDSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  /** Creates a new TurretAutoAimCommand. */
  public TurretAutoAimCommand(TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    addRequirements(m_turret, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLimelightOn(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.getTv() == true) {
      m_turret.setTargetAngle(m_turret.getCurrentAngle() + 4096.0/360.0 * m_limelight.getTx());
    }
    else {
      m_turret.stop();
    }
    System.out.println("getTx(): " + m_limelight.getTx());
    System.out.println("getCurrentAngle(): " + m_turret.getCurrentAngle());
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
