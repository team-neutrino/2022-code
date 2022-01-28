// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;
  private NetworkTableEntry m_shooterSpeed;
  private NetworkTableEntry m_setShooterRPM;
  private ShooterSubsystem m_shooter;
  private HttpCamera LLFeed;
  private static double testRPM;

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem(ShooterSubsystem p_shooter) {
    m_shooter = p_shooter;

    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    m_shooterSpeed = m_drivestationTab.add("Shooter RPM", 0).withPosition(0, 0).withSize(2, 2).withWidget(
    BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 6000)).getEntry();
    m_setShooterRPM = m_drivestationTab.add("Set Shooter RPM", 0).withPosition(0, 1).getEntry();


    LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg",HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(LLFeed); 
    m_drivestationTab.add(LLFeed).withPosition(1, 0).withSize(3, 2).withWidget(BuiltInWidgets.kCameraStream);
    m_drivestationTab.add(CameraServer.startAutomaticCapture()).withPosition(7, 0).withSize(7, 7).withWidget(BuiltInWidgets.kCameraStream);
    
  }

  @Override
  public void periodic() 
  {
    testRPM = m_setShooterRPM.getDouble(0.0);
    // This method will be called once per scheduler run
    m_shooterSpeed.setDouble(m_shooter.getRPM());

  }

  public double getTestRPM()
  {
    return testRPM;
  }
}
