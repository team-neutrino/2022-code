// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightSubsystem extends SubsystemBase {
  NetworkTable limelight;
  NetworkTableEntry ledMode;
  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ledMode = limelight.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turnOnLimelight();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void turnOnLimelight()
  {
    ledMode.setNumber(3);
  }

  public double getTx()
  {
    NetworkTableEntry tx = limelight.getEntry("tx");
    return tx.getDouble(0.0);
  }

  public double getTy()
  {
    NetworkTableEntry ty = limelight.getEntry("tx");
    return ty.getDouble(0.0);
  }

  public double getTa()
  {
    NetworkTableEntry ta = limelight.getEntry("tx");
    return ta.getDouble(0.0);
  }

  public boolean getTv()
  {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(0.0);
    if(validTarget == 1)
    {
      return true;
    }
    return false;
  }
}
