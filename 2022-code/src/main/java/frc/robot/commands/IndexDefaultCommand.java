// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IndexSubsystem;

public class IndexDefaultCommand extends CommandBase {
  /** Creates a new IndexMotorCommand. */
  private IndexSubsystem m_index;
  private ColorSubsystem m_color;

  public IndexDefaultCommand(IndexSubsystem p_index, ColorSubsystem p_color) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_index = p_index;
    m_color = p_color;
    addRequirements(p_index, p_color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("beam break " + m_index.getBeamBreak());
    System.out.println("proximity " + m_color.isBall());

    if(m_index.getBeamBreak())
    {
      m_index.MotorOneStop();
    }
    else if (m_color.isBall()) {
      m_index.MotorOneStart();
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
