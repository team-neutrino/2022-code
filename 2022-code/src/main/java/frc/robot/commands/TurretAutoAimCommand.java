// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;


public class TurretAutoAimCommand extends CommandBase {

  TurretSubsystem m_turret;
  LimelightSubsystem m_Limelight;

  /** Creates a new TurretAutoAimCommand. */
  public TurretAutoAimCommand(TurretSubsystem p_turret, LimelightSubsystem p_Limelight) {
    m_turret = p_turret;
    m_Limelight = p_Limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret, m_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.setLimelightOn(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (m_Limelight.getTv())
      {
        m_turret.setSetpoint(m_turret.getCurrentAngle() + m_Limelight.getTx());
      }
      else
      {
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
