// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;
  private ShuffleboardTab m_troubleshootTab; 

  private NetworkTableEntry m_limelightFeed;
  private HttpCamera feed;

  private NetworkTableEntry m_turretCurrentAngle;

  private TurretSubsystem m_turret;

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem(TurretSubsystem p_turret) {
    m_turret = p_turret;

    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");

    //feed = new HttpCamera("limelight", "http:///limelight.local:5800/stream.mjpg");
    
    m_turretCurrentAngle = m_drivestationTab.add("Turret Current Angle", 0).withPosition(6, 0).withSize(2, 2).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_turretCurrentAngle.setDouble(m_turret.getCurrentAngle());
  }
}
