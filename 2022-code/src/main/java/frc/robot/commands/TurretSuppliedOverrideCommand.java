// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;

public class TurretSuppliedOverrideCommand extends CommandBase {
  TurretPIDSubsystem m_turret;
  DoubleSupplier m_doubleSupplier;
  /** Creates a new TurretSuppliedOverrideCommand. */
  public TurretSuppliedOverrideCommand(TurretPIDSubsystem p_turret, DoubleSupplier p_doubleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_turret);
    m_turret = p_turret;
    m_doubleSupplier = p_doubleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setPower(0.5 * m_doubleSupplier.getAsDouble());
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
