// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShootWhileMove;

public class LimelightSubsystem extends SubsystemBase {
  private static final String DistanceThree = null;
  NetworkTable limelight;
  NetworkTableEntry ledMode;
  double h = 1.25;
  int cycles = 0;
  // needed?? double deltaX;
  double deltaTx;
  double limelightMountAngle = 30;
  double currentDistance;
  ShootWhileMove shooterUpdatorHelper;

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
    // shooterUpdatorHelper.integrateAngularV();
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

 /* public void deltaX() {
    System.out.println(cycles);
    double distance = getDistance();
    double deltaX = 0;
    double deltaXTwo = 0;
    double deltaXThree = 0;
    double distanceOne = 0;
    double distanceTwo = 0; 
    double distanceThree = 0;
    double distanceFour = 0;
    double distanceFive = 0;
    double distanceSix = 0;

    distanceSix = distanceFive;
    distanceFive = distanceFour;
    distanceFour = distanceThree;
    distanceThree = distanceTwo;
    distanceTwo = distanceOne;
    distanceOne = distance;

    deltaX = distanceOne - distanceTwo;
    deltaXTwo = distanceThree - distanceFour;
    deltaXThree = distanceFive - distanceSix;


    this.deltaX = (deltaX + deltaXTwo  + deltaXThree) / 3;
  }

  public double getDeltaX(){
    return deltaX;
  }
  */

  public void deltaTx() {
    double angle = getTx();
    double deltaTx = 0;
    double deltaTxTwo = 0;
    double deltaTxThree = 0;
    double angleOne = 0;
    double angleTwo = 0;
    double angleThree = 0;
    double angleFour = 0;
    double angleFive = 0;
    double angleSix = 0;

    angleSix = angleFive;
    angleFive = angleFour;
    angleFour = angleThree;
    angleThree = angleTwo;
    angleTwo = angleOne;
    angleOne = angle;

    deltaTx = angleOne - angleTwo;
    deltaTxTwo = angleThree - angleFour;
    deltaTxThree = angleFive - angleSix;

    this.deltaTx = (deltaTx + deltaTxTwo + deltaTxThree) / 3;
  }

  public double getDeltaTx(){
    return deltaTx;
  }

  public void distanceAngleUpdater() {
    if (cycles % 10 == 0){
      deltaTx();
    }
   
    
  }

}
