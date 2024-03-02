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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(VisionSub.class.getName());

  private final NetworkTable m_limelight;
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Vision");
  private final GenericEntry m_apriltagIDs, m_shuffleboardtx,
      m_shuffleboardty, m_shuffleboardta,
      m_target, m_tagID,
      m_apriltagCount, m_targetApriltag_sf;

  // private NetworkTableEntry m_tx;
  // private NetworkTableEntry m_ty;
  // private NetworkTableEntry m_ta;
  // private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tid;
  private NetworkTableEntry m_botpose_target;
  private NetworkTableEntry m_getpipe;
  private NetworkTableEntry m_pipeline; // Use constants for pipeline
  private boolean m_isRedAlliance = true;
  private int m_targetApriltagID = 99; // randome default value

  private LimelightHelpers.LimelightResults m_llresults;

  /** Creates a new VisionSub. */
  public VisionSub() {
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // m_tx = m_limelight.getEntry("tx");
    // m_ty = m_limelight.getEntry("ty");
    // m_ta = m_limelight.getEntry("ta");
    // m_tv = m_limelight.getEntry("tv");
    m_tid = m_limelight.getEntry("tid");
    m_botpose_target = m_limelight.getEntry("botpose_targetspace");
    m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");

    m_shuffleboardtx = m_shuffleboardTab.add("tx", 0).getEntry();
    m_shuffleboardty = m_shuffleboardTab.add("ty", 0).getEntry();
    m_shuffleboardta = m_shuffleboardTab.add("ta", 0).getEntry();
    m_target = m_shuffleboardTab.add("has target", false).getEntry();
    m_tagID = m_shuffleboardTab.add("get primary", 0).getEntry();
    m_apriltagCount = m_shuffleboardTab.add("apriltag count", 0).getEntry();
    m_apriltagIDs = m_shuffleboardTab.add("apriltag IDs", "").getEntry();
    m_targetApriltag_sf = m_shuffleboardTab.add("target apriltag", 0).getEntry();

    setTarget(2);
  }

  public void init() {
    m_logger.info("Initializing VisionSub");
    setPipeline(2); // Apriltag vision
  }

  @Override
  public void periodic() {
    m_llresults = LimelightHelpers.getLatestResults("");

    // This method will be called once per scheduler run
    updateShuffleBoard();
    //m_sbPivotPosition = m_shuffleboardTab.add("Pivot Position", 0).getEntry();
    // m_tx = m_shuffleboardTab.add("tx", 0).getEntry();
    // m_ty = m_shuffleboardTab.add("ty", 0).getEntry();
    // m_ta = m_shuffleboardTab.add("ta", 0).getEntry();

    // m_target = m_shuffleboardTab.add("has target", 0).getEntry();
    // m_tagID = m_shuffleboardTab.add("get primary", 0).getEntry();
  }

  public void setAlliance(boolean isRedAlliance) {
    m_isRedAlliance = isRedAlliance;
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
    int fiducial_count = m_llresults.targetingResults.targets_Fiducials.length;
    m_apriltagCount.setInteger(fiducial_count);

    String april_tag_ids = "";
    for(int j = 0; j < fiducial_count; j++) {
      LimelightTarget_Fiducial tag = m_llresults.targetingResults.targets_Fiducials[j];
      april_tag_ids += tag.fiducialID + " , ";
    }
    m_apriltagIDs.setString(april_tag_ids);

    // m_tx = m_limelight.getEntry("tx");
    // m_ty = m_limelight.getEntry("ty");
    // m_ta = m_limelight.getEntry("ta");
    // m_tv = m_limelight.getEntry("tv");
    // m_tid = m_limelight.getEntry("tid");
    double zero[] = {0.0};
    double pos[] = m_botpose_target.getDoubleArray(zero);
    SmartDashboard.putNumber("Apriltag ID", getPrimaryID());
    SmartDashboard.putNumber("Apriltag X", pos[0]);
    SmartDashboard.putNumber("Apriltag Y", pos[1]);
    SmartDashboard.putNumber("Apriltag Z", pos[2]);
    SmartDashboard.putNumber("Apriltag RX", pos[0]);
    SmartDashboard.putNumber("Apriltag RY", pos[1]);
    SmartDashboard.putNumber("Apriltag RZ", pos[2]);
    m_shuffleboardtx.setDouble(getHorizontalAngle());
    m_shuffleboardty.setDouble(getVerticalAngle());
    m_shuffleboardta.setDouble(getTargetArea());
    m_target.setBoolean(hasTarget());
    // m_tagID.setDouble(getPrimaryID());
    m_targetApriltag_sf.setInteger(m_targetApriltagID);
  }

  public void setTarget(int apriltagID) {
    m_targetApriltagID = apriltagID;
  }

  public boolean isTagInVision(int tagID) {

    int fiducial_count = m_llresults.targetingResults.targets_Fiducials.length;
    boolean tagPresent = false;
    for(int j = 0; j < fiducial_count; j++) {
      LimelightTarget_Fiducial tag = m_llresults.targetingResults.targets_Fiducials[j];
      if((int) tag.fiducialID == tagID) {
        tagPresent = true;
        break;
      }
    }
    return tagPresent;
  }

  private LimelightTarget_Fiducial getDetailsForTagId(int tagId) {
    int fiducial_count = m_llresults.targetingResults.targets_Fiducials.length;
    for(int j = 0; j < fiducial_count; j++) {
      LimelightTarget_Fiducial l_tag = m_llresults.targetingResults.targets_Fiducials[j];
      if((int) l_tag.fiducialID == tagId) {
        return l_tag;
      }
    }
    return null;
  }


  private double calcDistance(double height) { // Basic trig
    double theta = getVerticalAngle();
    theta = Math.toRadians(theta);
    double distance = height / Math.tan(theta);
    return distance;
  }

  public Pose3d getTarget3D() {
    double zero[] = {0.0};
    double pos[] = m_botpose_target.getDoubleArray(zero);
    Translation3d position = new Translation3d(pos[0], pos[1], pos[2]);
    Rotation3d rotation = new Rotation3d(pos[3], pos[4], pos[5]);
    return new Pose3d(position, rotation);
  }

  public double getHorizontalAngle() { // Horizontal offset between -27 to 27 degrees or -29.8 to 29.8 degrees
    if(hasTarget() == false) {
      return 0.0;
    }
    LimelightTarget_Fiducial tag = getDetailsForTagId(m_targetApriltagID);
    return tag.tx;

  }

  public double getVerticalAngle() { // Horizontal offset between -20.5 to 20.5 degrees or -24.85 to 24.85 degrees
    if(hasTarget() == false) {
      return -1;
    }
    LimelightTarget_Fiducial tag = getDetailsForTagId(m_targetApriltagID);
    return tag.ty;
  }

  public double getTargetArea() { // Get area of target in %
    if(hasTarget() == false) {
      return -1;
    }
    LimelightTarget_Fiducial tag = getDetailsForTagId(m_targetApriltagID);
    return tag.ta;
  }

  public boolean hasTarget() { // Returns true if any valid targets exist
    return isTagInVision(m_targetApriltagID);
    // return (m_tv.getDouble(0.0) == 0.0) ? false : true;
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

}
