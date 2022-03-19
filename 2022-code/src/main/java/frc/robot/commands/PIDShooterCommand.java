// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class PIDShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private int m_rpm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PIDShooterCommand(ShooterSubsystem p_shooter, int RPM) {
    m_shooter = p_shooter;
    m_rpm = RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.enable();
    m_shooter.setSetpoint(m_rpm);
    System.out.println(m_shooter.getOutput());
    // if (m_shooter.getOutput() < 0) {
    //   m_shooter.turnOff();
    // } else {
    //   m_shooter.setSetpoint(m_rpm);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
