// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(VisionSub.class.getName());


  private final NetworkTable m_limelight;
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;
  private NetworkTableEntry m_getpipe;
  private NetworkTableEntry m_pipeline; // Use constants for pipeline


  /** Creates a new VisionSub. */
  public VisionSub() {
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_limelight.getEntry("tx");
    m_ty = m_limelight.getEntry("ty");
    m_ta = m_limelight.getEntry("ta");
    m_tv = m_limelight.getEntry("tv");
    m_tid = m_limelight.getEntry("tid");
    m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("tx", getHorizontalAngle());
    SmartDashboard.putNumber("ty", getVerticalAngle());
    //SmartDashboard.putNumber("ta", getTargetArea());
    SmartDashboard.putBoolean("Target", hasTarget());
    SmartDashboard.putNumber("Tag ID", getPrimaryID());
    // Uncomment later but it's confusing right nowSmartDashboard.putNumber("Distance(m)", getDistance(getPrimaryID()));
  }

  public double getDistance(int id) { // In meters
    if(id < 1 || id > 16) // Got bad apriltag
      return 0.0;
    double height = Constants.VisionConstants.kApriltagHeights[id + 1] + Constants.VisionConstants.kApriltagOffset;
    double distance = calcDistance(height);

    return distance;
  }

  private double calcDistance(double height) { // Basic trig
    double theta = getVerticalAngle();
    theta = Math.toRadians(theta);
    double distance = height / Math.tan(theta);
    return distance;
  }

  public double getHorizontalAngle() { // Horizontal offset between -27 to 27 degrees or -29.8 to 29.8 degrees
    return m_tx.getDouble(0.0);
  }

  public double getVerticalAngle() { // Horizontal offset between -20.5 to 20.5 degrees or -24.85 to 24.85 degrees
    return m_ty.getDouble(0.0);
  }

  public double getTargetArea() { // Get area of target in %
    return m_ta.getDouble(0.0);
  }

  public boolean hasTarget() { // Returns true if any valid targets exist
    return (m_tv.getDouble(0.0) == 0.0) ? false : true;
  }

  public int getVisionMode() { // Gets current vision pipeline number
    double val = m_getpipe.getDouble(0.0);
    return (int) val;
  }

  public int getPrimaryID() { // Get primary apriltag ID (-1 means nothing)
    Long val = m_tid.getInteger(-1);
    return val.intValue();
  }

  public void setPipeline(int line) { // Set the currect pipeline (NO_VISION, LIMELIGHT, or APRILTAG)
    m_pipeline.setNumber(line);
  }

  public void init() {

  }
}
