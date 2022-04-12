// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new InterpolatedShooterSpeed. */
  ShooterSubsystem m_shooter;
  LimelightSubsystem m_limelight;
  double m_RPM;

  public AutoShootCommand(ShooterSubsystem p_shooter, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_limelight = p_limelight;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.resetCounter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RPM = m_shooter.CalculateRPM();
    if(m_limelight.getTv() && m_shooter.okShoot())
    {
        m_shooter.setTargetRPM(m_shooter.CalculateRPM());
        m_shooter.iterateCounter(m_shooter.CalculateRPM());
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
