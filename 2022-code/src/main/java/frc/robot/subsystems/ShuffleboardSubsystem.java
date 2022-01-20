// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;
  private NetworkTableEntry m_testOutput;  
  private HttpCamera LLFeed;

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem() {
    System.out.println("Wineinger");
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    try {
    LLFeed = new HttpCamera("limelight", "http://10.39.28.12:5800/stream.mjpg",
      HttpCameraKind.kMJPGStreamer);
    // m_drivestationTab.add(LLFeed);
    CameraServer.startAutomaticCapture(LLFeed);
    m_drivestationTab.add(LLFeed).withPosition(1, 0).withSize(3, 2).withWidget(BuiltInWidgets.kCameraStream);
    System.out.println("cale"); }
    catch(VideoException e) {
      System.out.println("swag");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
