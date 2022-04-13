// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.util.Vector;

public class TurretAutoAimCommand extends CommandBase {
  private TurretPIDSubsystem m_turret;
  private LimelightSubsystem m_limelight;
  private DriveTrainSubsystem m_drive;
  private double LIMELIGHT_MULTIPLICATION = 10.0;

  private double r0;
  private double x0;
  private double y0;
  private double vel;
  private Vector hubVec;
  private Vector R0;
  /** Creates a new  TurretAutoAimCommand. */
  public TurretAutoAimCommand(TurretPIDSubsystem p_turret, LimelightSubsystem p_limelight, DriveTrainSubsystem p_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_limelight = p_limelight;
    m_drive = p_drive;
    addRequirements(m_turret, m_limelight, m_drive);

    r0 = m_limelight.getDistance();
    x0 = m_drive.getDriveEncoderL1Position();
    y0 = m_drive.getDriveEncoderR1Position();
    hubVec = m_limelight.getHubVec();
    R0 = (hubVec.inverse()).addWith(new Vector(x0, y0));
    vel = (m_drive.getDriveEncoderL1() + m_drive.getDriveEncoderR1()) / 2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setLimelightOn();
  }

  private double getAdjustAngle() {
    double dx = m_drive.getNavX() - x0;
    double dy = m_drive.getNavY() - y0;
    Vector S = new Vector(dx, dy);
    Vector R1 = R0.addWith(S);
    double theta = Vector.argFromLOC(R1.getNorm(), r0, S.getNorm());

    Vector V = new Vector(vel * dx / S.getNorm(), vel * dy / S.getNorm());
    Vector velTraj = R1.addWith(V);
    double phi = R1.arg(velTraj);

    double finalAdj = theta + phi;
    return finalAdj;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    m_limelight.setLimelightOn();
    if (m_limelight.getTv() == true) {
      m_turret.setTargetAngle(
          m_turret.getCurrentAngle() + LIMELIGHT_MULTIPLICATION * m_limelight.getTx());
    } else {
      m_turret.stop();
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
}
