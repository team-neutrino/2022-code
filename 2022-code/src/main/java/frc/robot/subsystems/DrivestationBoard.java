// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivestationBoard extends SubsystemBase {

  private ShuffleboardTab m_drivestationTab;

  private NetworkTableEntry m_test_joystickPosition;  
  private NetworkTableEntry m_user;

  /** Creates a new shuffleboard. */
  public DrivestationBoard() {
    m_drivestationTab = Shuffleboard.getTab("Drivestation Tab");
    m_test_joystickPosition = m_drivestationTab.add("Joystick position", 2)
      .withPosition(0, 0)
      .withSize(2, 2)
      .getEntry();   
    m_user = m_drivestationTab.add("Username", "neutrino")
      .withPosition(2, 0)
      .withSize(2, 2)
      .getEntry();

  }

  public String getUsername() {
    return m_user.getString("neutrino");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_test_joystickPosition.setDouble(2);
  }
}
