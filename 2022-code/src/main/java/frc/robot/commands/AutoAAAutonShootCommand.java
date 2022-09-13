// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

/** An example command that uses an example subsystem. */
public class AutoAAAutonShootCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private IndexSubsystem m_index;
  private LimelightSubsystem m_limelight;
  private TurretPIDSubsystem m_turret;
  private Timer m_timer;
  private double m_duration;
  private double m_spinUp;
  private double m_RPM;
  private double LIMELIGHT_MULTIPLICATION = 20.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAAAutonShootCommand(
      ShooterSubsystem p_shooter,
      IndexSubsystem p_index,
      TurretPIDSubsystem p_turret,
      LimelightSubsystem p_limelight,
      double p_duration) {
    m_shooter = p_shooter;
    m_index = p_index;
    m_turret = p_turret;
    m_limelight = p_limelight;
    m_timer = new Timer();
    m_duration = p_duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLimelightOn();
    m_shooter.resetCounter();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.getTv() == true) {
      m_turret.setTargetAngle(
          m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx());
    } else {
      m_turret.stop();
    }

    m_RPM = m_shooter.CalculateRPM();
    m_shooter.setTargetRPM(m_RPM + 30);
    m_shooter.iterateCounter(m_RPM + 30);

    if (m_shooter.okShoot()) {
      m_index.MotorOneStart();
      m_index.MotorTwoStart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.MotorOneStop();
    m_index.MotorTwoStop();
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() >= m_duration) {
      return true;
    }
    return false;
  }
}
