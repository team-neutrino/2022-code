// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class AutonShootCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private IndexSubsystem m_index;
  private Timer m_timer;
  private double m_duration;
  private int m_RPM;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonShootCommand(
      ShooterSubsystem p_shooter, IndexSubsystem p_index, int p_RPM, double p_duration) {
    m_shooter = p_shooter;
    m_index = p_index;
    m_timer = new Timer();
    m_duration = p_duration;
    m_RPM = p_RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTargetRPM(m_RPM);
    if (m_timer.get() >= 1.5) {
      m_index.MotorOneStart();
      m_index.MotorTwoStart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.MotorOneStop();
    m_index.MotorTwoStop();
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= m_duration) {
      return true;
    }
    return false;
  }
}
