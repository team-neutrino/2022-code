// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import java.util.function.DoubleSupplier;

public class TurretSuppliedOverrideCommand extends CommandBase {
  TurretPIDSubsystem m_turret;
  LimelightSubsystem m_limelight;
  DoubleSupplier m_doubleSupplier;
  boolean m_setLimelightOn;
  /** Creates a new TurretSuppliedOverrideCommand. */
  public TurretSuppliedOverrideCommand(
      TurretPIDSubsystem p_turret,
      LimelightSubsystem p_limelight,
      DoubleSupplier p_doubleSupplier,
      boolean p_setLimelightOn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    addRequirements(m_turret, m_limelight);
    m_doubleSupplier = p_doubleSupplier;
    m_setLimelightOn = p_setLimelightOn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_setLimelightOn) {
      m_limelight.setLimelightOn();
    } else {
      m_limelight.setLimelightOff();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setPower(0.5 * m_doubleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLimelightOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
