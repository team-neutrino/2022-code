// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;
  private ShuffleboardTab m_troubleshootTab; 
  private HttpCamera limelightFeed;


  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem() {
    /** m_driverstation widgets */
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");

    /**m_troubleshoot widgets */
    m_troubleshootTab = Shuffleboard.getTab("Troubleshoot Tab");
    limelightFeed = new HttpCamera("limelight", "http:///limelight.local:5800/stream.mjpg");
    m_troubleshootTab.add("limelight feed", limelightFeed).withPosition(0, 0).withSize(8, 8);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
