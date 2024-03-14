// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class VisionSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(VisionSub.class.getName());

  private final NetworkTable m_limelight;
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Vision");
  private final GenericEntry m_shuffleboardtx,
      m_shuffleboardty,
      m_target,
      m_apriltagCount;

  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;
  private NetworkTableEntry m_botpose_target;
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
    m_botpose_target = m_limelight.getEntry("botpose_targetspace");
    m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");

    m_shuffleboardtx = m_shuffleboardTab.add("tx", 0).getEntry();
    m_shuffleboardty = m_shuffleboardTab.add("ty", 0).getEntry();

    m_target = m_shuffleboardTab.add("has target", false).getEntry();
    m_apriltagCount = m_shuffleboardTab.add("apriltag count", 0).getEntry();
  }

  public void init() {
    m_logger.info("Initializing VisionSub");
    setPipeline(1); // Apriltag vision
  }

  @Override
  public void periodic() {

    if(!RobotContainer.disableShuffleboardPrint) {
      // This method will be called once per scheduler run
      updateShuffleBoard();
    }
  }

  public double getDistance(int id) { // In meters
    if(id < 1 || id > 16) // Got bad apriltag
      return 0.0;
    double height = Constants.Vision.kApriltagHeights[id + 1] + Constants.Vision.kApriltagOffset;
    double distance = calcDistance(height);

    return distance;
  }

  public void updateShuffleBoard() {
    // Ref: https://docs.limelightvision.io/docs/docs-limelight/apis/json-dump-specification#apriltagfiducial-results
    // NOTE: (from doc) :  Takes up to 2.5ms on RoboRIO 1.0.

    m_apriltagCount.setInteger(0);

    //SmartDashboard.putNumber("Apriltag RY", Math.floor(getTargetRotation().getY()));
    m_shuffleboardtx.setDouble(getSimpleHorizontalAngle());
    m_shuffleboardty.setDouble(getSimpleVerticalAngle());
    m_target.setBoolean(simpleHasTarget());
  }


  private double calcDistance(double height) { // Basic trig
    double theta = getSimpleVerticalAngle();
    theta = Math.toRadians(theta);
    double distance = height / Math.tan(theta);
    return distance;
  }

  public Pose3d getTarget3D() {
    double pos[] = m_botpose_target.getDoubleArray(new double[6]);
    Translation3d position = new Translation3d(pos[0], pos[1], pos[2]);
    Rotation3d rotation = new Rotation3d(pos[3], pos[4], pos[5]);
    return new Pose3d(position, rotation);
  }

  public Translation3d getTargetRotation() {
    double pos[] = m_botpose_target.getDoubleArray(new double[6]);
    Translation3d rotation = new Translation3d(pos[3], pos[4], pos[5]);
    return rotation;
  }

  public double getSimpleHorizontalAngle() {
    return m_tx.getDouble(0.0);
  }

  public double getSimpleVerticalAngle() {
    return m_ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return m_ta.getDouble(0.0);
  }

  public boolean simpleHasTarget() {
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

  public void setPipeline(int line) { // Set the currect pipeline (NO_VISION, APRILTAG, or APRILTAG 3x)
    m_pipeline.setNumber(line);
  }

}
