// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new InterpolatedShooterSpeed. */
  ShooterSubsystem m_shooter;

  DriveTrainSubsystem m_drive;
  final double m_angleTolerance = 1;
  final double m_driveEncoderTolerance = 10;
  IndexSubsystem m_index;
  LimelightSubsystem m_limelight;
  double m_RPM;
  Timer m_timer;

  public AutoShootCommand(
      ShooterSubsystem p_shooter,
      IndexSubsystem p_index,
      DriveTrainSubsystem p_drive,
      LimelightSubsystem p_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_drive = p_drive;
    m_timer = new Timer();
    m_index = p_index;
    m_limelight = p_limelight;
    addRequirements(m_shooter, m_index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RPM = m_shooter.CalculateRPM();
    m_shooter.setTargetRPM(m_RPM);
    m_shooter.iterateCounter(m_RPM);

    System.out.println(
        "valid target: " + m_limelight.getTv() + "  shooter ok shoot: " + m_shooter.okShoot());

    // check that we are within shooting range
    if (shoot()) {
      m_index.MotorOneStart();
      m_index.MotorTwoStart();
    } else {
      m_index.MotorOneStop();
      m_index.MotorTwoStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean shoot() {
    return m_drive.getDriveEncoderR1() < m_driveEncoderTolerance
        && m_drive.getDriveEncoderL1() < m_driveEncoderTolerance
        && m_limelight.getTv()
        && Math.abs(m_limelight.getTx()) < m_angleTolerance
        && m_shooter.okShoot();
  }
}
