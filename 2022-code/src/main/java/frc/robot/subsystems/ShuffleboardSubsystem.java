// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class ShuffleboardSubsystem extends SubsystemBase 
{

  private ShuffleboardTab m_drivestationTab;
  private NetworkTableEntry m_shooterSpeed;
  private NetworkTableEntry m_setShooterRPM;
  private ShooterSubsystem m_shooter;
  private HttpCamera LLFeed;
  private NetworkTableEntry m_turretAngle;
  private TurretPIDSubsystem m_turret;
  private NetworkTableEntry m_timer;

  

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem(ShooterSubsystem p_shooter, TurretPIDSubsystem p_turret) {
    m_shooter = p_shooter;
    m_turret = p_turret;

    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    m_shooterSpeed = m_drivestationTab.add("Shooter RPM", 0).withPosition(0, 0).withSize(2, 2).withWidget(
    BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 6000)).getEntry();
    m_setShooterRPM = m_drivestationTab.add("Set Shooter RPM", 0).withPosition(0, 1).getEntry();
    m_timer = m_drivestationTab.add("Match Time", 0 ).withPosition(5, 5).withSize(8, 8).getEntry();

    
    try
    {
      LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg",HttpCameraKind.kMJPGStreamer);
      CameraServer.startAutomaticCapture(LLFeed); 
      m_drivestationTab.add(LLFeed).withPosition(1, 0).withSize(3, 2).withWidget(BuiltInWidgets.kCameraStream);
      m_drivestationTab.add(CameraServer.startAutomaticCapture()).withPosition(9, 0).withSize(7, 7).withWidget(BuiltInWidgets.kCameraStream);
    }
    catch(VideoException e) 
    {

    }
    m_turretAngle = m_drivestationTab.add("Turret Angle", 6).withPosition(5, 0).withSize(2, 2).getEntry();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    m_shooterSpeed.setDouble(m_shooter.getRPM());
    m_turretAngle.setDouble(m_turret.getCurrentAngle());
    m_timer.setDouble(DriverStation.getMatchTime());
  }

  public double getTestRPM()
  {
    return m_setShooterRPM.getDouble(0.0);
  }
}
