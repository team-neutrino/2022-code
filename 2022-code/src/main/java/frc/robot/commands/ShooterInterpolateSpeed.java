// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterInterpolateSpeed extends CommandBase {
  /** Creates a new InterpolatedShooterSpeed. */
  ShooterSubsystem m_shooter;

  boolean m_driverControl;
  double m_RPM;

  public ShooterInterpolateSpeed(ShooterSubsystem p_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_driverControl = false;
    addRequirements(m_shooter);
  }

  public ShooterInterpolateSpeed(ShooterSubsystem p_shooter, boolean p_driverControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_driverControl = p_driverControl;
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

    if (m_driverControl) {
      m_shooter.setCounter(10);
      m_shooter.setTargetRPM(m_RPM);
    } else {
      m_shooter.setTargetRPM(m_RPM);
      m_shooter.iterateCounter(m_RPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
