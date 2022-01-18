// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretManualAimCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private boolean m_isClockwise;
  private int m_negativeConstant;
  /** Creates a new ContinuousTurnCommand. */
  public TurretManualAimCommand(TurretSubsystem p_turret, boolean isClockwise) {
    m_turret = p_turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
    m_isClockwise = isClockwise;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isClockwise) {
      m_turret.setSetpoint(m_turret.getCurrentAngle() + Constants.TurretConstants.TURRET_UPDATE_ANGLE);
    }
    else {
      m_turret.setSetpoint(m_turret.getCurrentAngle() - Constants.TurretConstants.TURRET_UPDATE_ANGLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
