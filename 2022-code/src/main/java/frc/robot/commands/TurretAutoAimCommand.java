// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAutoAimCommand extends CommandBase {

  private TurretSubsystem m_turret;
  private limelightSubsystem m_limelight;

  /** Creates a new TurretAutoAimCommand. */
  public TurretAutoAimCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
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
    if (m_limelight.getTv())
    {
      m_turret.setSetpoint(m_turret.getCurrentAngle() + m_limelight.getTx());
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
