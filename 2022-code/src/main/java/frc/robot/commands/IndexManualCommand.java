// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexManualCommand extends CommandBase {
  /** Creates a new IndexManualCommand. */
  private IndexSubsystem m_index;

  private ShooterSubsystem m_shooter;
  private LimelightSubsystem m_limelight;

  public IndexManualCommand(
      IndexSubsystem p_index, ShooterSubsystem p_shooter, LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_index = p_index;
    m_shooter = p_shooter;
    m_limelight = p_limelight;
    addRequirements(m_index, m_shooter, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_limelight.getTv());
    if (m_shooter.okShoot()) {
      m_index.MotorOneStart();
      m_index.MotorTwoStart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.MotorOneStop();
    m_index.MotorTwoStop();
    m_shooter.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
