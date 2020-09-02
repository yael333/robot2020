/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * @author yuval rader
 */

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase {
  public double x;
  public double y;
  public boolean v;
  public double a;
  public double s;
  public double l;
  public double Tshort;
  public double Tlong;
  public double Thor;
  public double Tvert;
  public double Pipe;
  public double yaw;
  public double distanceFromTargetLimelightX;
  public double distanceFromTargetLimelightY;
  public double pipe;

  public NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry threeDimension;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry ts;
  private NetworkTableEntry tl;
  private NetworkTableEntry tlong;
  private NetworkTableEntry tshort;
  private NetworkTableEntry thor;
  private NetworkTableEntry tvert;
  private NetworkTableEntry getpipe;

  private static limelight limelight;

  /**
   * Creates a new Limlight.
   */
  private limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    tl = table.getEntry("tl");
    ts = table.getEntry("ts");
    tshort = table.getEntry("tshort");
    tlong = table.getEntry("tlong");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
    getpipe = table.getEntry("getpipe");
    threeDimension = table.getEntry("camtran");
  }

  public void ledMode(int ledMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }

  public void camMode(int camMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
  }

  public void pipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(pipeline);
  }

  public void stream(int stream) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(stream);
  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);
    v = tv.getBoolean(false);
    s = ts.getDouble(0.0);
    l = tl.getDouble(0.0);
    pipe = getpipe.getDouble(0.0);
    Tlong = tlong.getDouble(0.0);
    Thor = thor.getDouble(0.0); 
    Tvert = tvert.getDouble(0.0);
    Tshort = tshort.getDouble(0.0);
    yaw = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0 })[4];
    distanceFromTargetLimelightX = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[0];
    distanceFromTargetLimelightY = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[2];
    
  }

  public static limelight getinstance() {
    if (limelight == null) {
      limelight = new limelight();
    }
    return limelight;
  }
}
