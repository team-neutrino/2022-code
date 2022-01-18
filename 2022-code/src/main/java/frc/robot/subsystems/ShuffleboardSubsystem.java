// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;
  private NetworkTableEntry m_testOutput;  

  /** Creates a new shuffleboard. */
  public ShuffleboardSubsystem() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_testOutput.setDouble(2);
  }
}
