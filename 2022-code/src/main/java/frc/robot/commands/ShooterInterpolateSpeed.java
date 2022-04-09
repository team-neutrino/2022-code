// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.CalculateRPM;

public class ShooterInterpolateSpeed extends CommandBase {
  /** Creates a new InterpolatedShooterSpeed. */
  ShooterSubsystem m_shooter;

  public ShooterInterpolateSpeed(ShooterSubsystem p_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = p_shooter;

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
    System.out.println("target rpm: " +m_shooter.CalculateRPM());
    System.out.println("actual rpm: " + m_shooter.getRPM1());

    m_shooter.setTargetRPM(m_shooter.CalculateRPM());
    m_shooter.iterateCounter(m_shooter.CalculateRPM());
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
