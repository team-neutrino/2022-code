// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;

public class AutonTurretCommand extends CommandBase {

  private TurretPIDSubsystem m_turret;
  private double m_angle;

  /** Creates a new TurretManualAimCommand. */
  public AutonTurretCommand(TurretPIDSubsystem p_turret, double p_angle) {
    m_turret = p_turret;
    m_angle = p_angle;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.turnCounterClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_turret.getCurrentAngle() >= m_angle) {
      return true;
    }
    return false;
  }
}
