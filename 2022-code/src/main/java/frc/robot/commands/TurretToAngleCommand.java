// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class TurretToAngleCommand extends CommandBase {
  private TurretPIDSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  private double m_setpointAngle;
  private double m_initialAngle;

  /** Creates a new TurretAutoAimCommand. */
  public TurretToAngleCommand(
      TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight, double p_setpointAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    addRequirements(m_turret, m_limelight);

    m_setpointAngle = p_setpointAngle;
    m_initialAngle = m_turret.getInitialAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_limelight.setLimelightOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setTargetAngle(m_setpointAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_limelight.setLimelightOn();
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
