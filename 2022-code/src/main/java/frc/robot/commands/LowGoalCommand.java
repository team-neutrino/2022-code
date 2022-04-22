// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class LowGoalCommand extends CommandBase {
  /** shooter rpm constant */
  private ShooterSubsystem m_shooter;

  private TurretPIDSubsystem m_turret;

  double m_rpm;

  /** Creates a new ShooterSetSpeedCommand. */
  public LowGoalCommand(ShooterSubsystem p_shooter, TurretPIDSubsystem p_turret, double rpm) {
    m_shooter = p_shooter;
    m_turret = p_turret;
    m_rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setCounter(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setTargetAngle(320);
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
