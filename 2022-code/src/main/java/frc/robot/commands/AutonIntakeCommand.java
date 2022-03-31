// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;

/** An example command that uses an example subsystem. */
public class AutonIntakeCommand extends CommandBase {
  private IntakeSubSystem m_intake;
  private Timer m_timer;
  private double m_duration;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonIntakeCommand(IntakeSubSystem p_intake, double p_duration) {
    m_intake = p_intake;
    m_timer = new Timer();
    m_duration = p_duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setDown();
    m_intake.setIntakeOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_intake.setUp();
    m_intake.setIntakeOff();
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
