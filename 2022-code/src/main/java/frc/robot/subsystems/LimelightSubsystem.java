// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable limelight;
  NetworkTableEntry ledMode;
  double h = 1.25;
  double limelightMountAngle = 30;

  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ledMode = limelight.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setLimelightOn() {
    ledMode.setNumber(3);
  }

  public void setLimelightOff() {
    ledMode.setNumber(1);
  }

  public double getTx() {
    NetworkTableEntry tx = limelight.getEntry("tx");
    return tx.getDouble(0.0);
  }

  public double getTy() {
    NetworkTableEntry ty = limelight.getEntry("ty");
    return ty.getDouble(0.0);
  }

  public double getTa() {
    NetworkTableEntry ta = limelight.getEntry("ta");
    return ta.getDouble(0.0);
  }

  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(0.0);
    if (validTarget == 1) {
      return true;
    }
    return false;
  }

  public double getDistance() {
    return h / Math.tan(Math.toRadians(getTy() + limelightMountAngle));
  }
}
