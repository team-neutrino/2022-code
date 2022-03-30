// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class IntakeRedBallOutCommand extends CommandBase {
  private IndexSubsystem m_index;
  private IntakeSubSystem m_intake;
  private Timer m_timer;
  private double K_OUTTAKE_TIME = 5.0;
  /** Creates a new RedBallOutCommand. */
  public IntakeRedBallOutCommand(IndexSubsystem p_index, IntakeSubSystem p_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_index = p_index;
    m_intake = p_intake;
    addRequirements(m_index, m_intake);

    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_index.motorOneBack();
    m_index.motorTwoBack();
    m_intake.setIntakeReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_index.motorOneStop();
    m_index.motorTwoStop();
    m_intake.setIntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > K_OUTTAKE_TIME)
      return true;
    else
      return false;
  }
}
