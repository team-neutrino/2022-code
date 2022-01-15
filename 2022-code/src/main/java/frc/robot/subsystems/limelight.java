// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelight extends SubsystemBase {
    NetworkTable table;
  public limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }

  public double getTx()
  {
    return table.getEntry("tx").getDouble(0);
  }

  public double getTy()
  {
    return table.getEntry("ty").getDouble(0);
  }

  public double getTa()
  {
    return table.getEntry("ta").getDouble(0);
  }
}
