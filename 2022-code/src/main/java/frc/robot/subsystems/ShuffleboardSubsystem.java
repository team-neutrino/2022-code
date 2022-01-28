// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase
{

    private ShuffleboardTab m_drivestationTab;
    private HttpCamera LLFeed;

    /** Creates a new shuffleboard. */
    public ShuffleboardSubsystem()
    {
        m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(LLFeed);
        m_drivestationTab.add(LLFeed).withPosition(1, 0).withSize(3, 2).withWidget(BuiltInWidgets.kCameraStream);
        m_drivestationTab.add(CameraServer.startAutomaticCapture()).withPosition(7, 0).withSize(7, 7).withWidget(
            BuiltInWidgets.kCameraStream);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
