// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretToAngleCommand extends CommandBase {
  /** Creates a new TurretSetAngleCommand. */
  private TurretSubsystem m_turret;
  private double m_targetAngle;
  
  public TurretToAngleCommand(TurretSubsystem p_turret, double p_targetAngle) {
    m_turret = p_turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
    m_targetAngle = p_targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setSetpoint(m_targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_turret.getCurrentAngle() - m_targetAngle) < Constants.TurretConstants.TURRET_DEAD_ANGLE) {
      return true;
    }
    else {
      return false;
    }
  }
}
