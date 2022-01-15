// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretContinuousTurnCommand extends CommandBase {
  private TurretSubsystem m_turret = new TurretSubsystem();
  private boolean m_isClockwise;
  private int m_negativeConstant;
  /** Creates a new ContinuousTurnCommand. */
  public TurretContinuousTurnCommand(boolean isClockwise) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);

    m_isClockwise = isClockwise;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_negativeConstant = (m_isClockwise)? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double augmentedAngle = m_negativeConstant * (m_turret.getCurrentAngle() + Constants.TurretConstants.TURRET_UPDATE_ANGLE);
    m_turret.setAngle(augmentedAngle);
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
