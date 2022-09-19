// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private static final String DistanceThree = null;
  NetworkTable limelight;
  NetworkTableEntry ledMode;
  double h = 1.25;
  int cycles = 0;
  double deltaD;
  double deltaA;
  double limelightMountAngle = 30;
  double currentDistance;

  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ledMode = limelight.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // currentDistance = getDistance();
    cycles++;
    distanceAngleUpdater();
    // getTime();
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

  public void deltaD() {
    double distance = getDistance();
    double distanceOne = 0;
    double distanceTwo = 0; 
    double distanceThree = 0;

    distanceThree = distanceTwo;
    distanceTwo = distanceOne;
    distanceOne = distance;

    this.deltaD = distanceOne - distanceTwo;
  }

  public double getDeltaD(){
    return deltaD;
  }

  public void deltaA() {
    double angle = getTx();
    double angleOne = 0;
    double angleTwo = 0;
    double angleThree = 0;

    angleThree = angleTwo;
    angleTwo = angleOne;
    angleOne = angle;

    this.deltaA = angleOne - angleTwo;
  }

  public double getDeltaA(){
    return deltaA;
  }

  public void distanceAngleUpdater() {
    if (cycles % 10 == 0){
      deltaD();
      deltaA();
    }
   
    
  }
  public double getTanV(){
    double tanV = getDeltaD() / 0.2;
    return tanV;
  }

}
