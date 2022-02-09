// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetSpeed extends CommandBase {
  /** shooter rpm constant */
  private final double SHOOTER_RPM_1 = 3000;

  private ShooterSubsystem m_shooter;
  private double m_rpm = SHOOTER_RPM_1;
  /** Creates a new ShooterSetSpeedCommand. */
  public ShooterSetSpeed(ShooterSubsystem p_shooter) {
    m_shooter = p_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTargetRPM(m_rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
