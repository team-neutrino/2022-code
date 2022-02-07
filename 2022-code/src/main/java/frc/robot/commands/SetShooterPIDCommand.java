// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterPIDCommand extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private double m_pValue;
  private double m_iValue;
  private double m_dValue;
  
  public SetShooterPIDCommand(ShooterSubsystem p_shooter, double p_pValue, double p_iValue, double p_dValue) {
    m_shooter = p_shooter;
    m_pValue = p_pValue;
    m_iValue = p_iValue;
    m_dValue = p_dValue;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_shooter.setP(m_pValue);
      m_shooter.setI(m_iValue);
      m_shooter.setD(m_dValue);
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
