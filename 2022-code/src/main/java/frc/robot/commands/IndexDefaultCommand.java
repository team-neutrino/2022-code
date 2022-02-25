// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class IndexDefaultCommand extends CommandBase {
  /** Creates a new IndexMotorCommand. */
  private IndexSubsystem m_index;

  public IndexDefaultCommand(IndexSubsystem p_index) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_index);
    m_index = p_index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_index.getBeamBreak()) {
      m_index.MotorOneStart();    } 
    else {
      m_index.MotorOneStop();
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
