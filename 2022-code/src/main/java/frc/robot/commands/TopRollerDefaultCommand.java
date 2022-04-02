// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class TopRollerDefaultCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private double m_targetRPM;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TopRollerDefaultCommand(ShooterSubsystem p_shotoer) {
    m_shooter = p_shotoer;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_targetRPM = m_shooter.CalculateRPM();
    m_shooter.setTopRollerRPM(m_targetRPM);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
