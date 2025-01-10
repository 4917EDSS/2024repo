// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.DriveToRelativePositionCmd;

public class VisionSub extends SubsystemBase {

  //Gets the limelight network table values (list of labels associated with values)
  /*
   * Important network table values:
   * tv: 1 If valid target exists. 0 if no valid targets exist
   * tx: Horizontal offset from crosshair to target (In degrees)
   * ty: Vertical offset from crosshair to target (In degrees)
   * ta: Target area, how much of the feed is covered by the limelight
   * tid: Id of the primary apriltag currently in view
   * botpose_targetspace: 3D transform from the primary in view apriltag
   * getpipe: Active camera pipeline index (0,9)
   */
  //Go to https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api for full list of network table values
  private final NetworkTable m_limelight;
  private final NetworkTable m_limelightDriver;
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Vision");
  private final GenericEntry m_shuffleboardtx,
      m_shuffleboardty,
      m_target,
      m_apriltagCount;

  private NetworkTableEntry m_tnx;
  private NetworkTableEntry m_tnv;


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
    //Gets the network table
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    m_limelightDriver = NetworkTableInstance.getDefault().getTable("limelight-driver");
    m_tnx = m_limelightDriver.getEntry("tx");
    m_tnv = m_limelightDriver.getEntry("tv");
    m_tx = m_limelight.getEntry("tx");
    m_ty = m_limelight.getEntry("ty");
    m_ta = m_limelight.getEntry("ta");
    m_tv = m_limelight.getEntry("tv");
    m_tid = m_limelight.getEntry("tid");
    m_botpose_target = m_limelight.getEntry("botpose_targetspace");
    m_getpipe = m_limelight.getEntry("getpipe");
    m_pipeline = m_limelight.getEntry("pipeline");

    //Adds values to the shuffleboard
    m_shuffleboardtx = m_shuffleboardTab.add("tx", 0).getEntry();
    m_shuffleboardty = m_shuffleboardTab.add("ty", 0).getEntry();

    m_target = m_shuffleboardTab.add("has target", false).getEntry();
    m_apriltagCount = m_shuffleboardTab.add("apriltag count", 0).getEntry();
  }

  public void init() {
    //Sets the apriltag pipeline
    setPipeline(1); // Apriltag vision
  }

  @Override
  public void periodic() {}

  //Method to get the apriltag heights from constants since not all have the same height, 
  public double getDistance(int id) { // In meters
    /*
     * if(id < 1 || id > 16) // Got bad apriltag, since the ID values used are between 1 and 16 (inclusive)
     * return 0.0;
     * double height = Constants.Vision.kApriltagHeights[id + 1] + Constants.Vision.kApriltagOffset;
     * //Calculates the distacne from the apriltag to the robot using the calcDistacne mehtod
     */

    double distance = calcDistance(0.17);
    return distance;
  }

  //Updates shuffleboard with apriltag values
  public void updateShuffleBoard() {
    // Ref: https://docs.limelightvision.io/docs/docs-limelight/apis/json-dump-specification#apriltagfiducial-results
    // NOTE: (from doc) :  Takes up to 2.5ms on RoboRIO 1.0.

    m_apriltagCount.setInteger(0);

    //SmartDashboard.putNumber("Apriltag RY", Math.floor(getTargetRotation().getY()));
    m_shuffleboardtx.setDouble(getSimpleHorizontalAngle());
    m_shuffleboardty.setDouble(getSimpleVerticalAngle());
    m_target.setBoolean(simpleHasTarget());
  }


  //Trig explanation: the tangent of an angle equals opposite/adjacent (opposite is the side not forming the angle, adjacent is the other side
  //which isn't the hypotenuse). If we call the angle θ, then tan(θ)=opposite/adjacent. If we know θ and the opposite (apriltag height), the we can re-arrange
  //to adjacent=opposite/tan(θ) which is teh equation we use.
  private double calcDistance(double height) { // Basic trig
    //Sets theta to the angle between the camera's crosshair and the april tag
    double theta = getSimpleVerticalAngle();
    //Converts theta to radians
    theta = Math.toRadians(theta);
    //Divides the height of the apriltag by the tangent of theta to get the distance from the april tag horizontally
    double distance = height / Math.tan(theta);
    return distance;
  }

  //Method to create a pose3D from the botpose_target network table value
  public Pose3d getTarget3D() {
    //Creates an array using the position values (0:2 are x,y,z, and 3:5 are yaw, roll, pitch. Not in those specific orders)
    double pos[] = m_botpose_target.getDoubleArray(new double[6]);
    Translation3d position = new Translation3d(pos[0], pos[1], pos[2]);
    Rotation3d rotation = new Rotation3d(pos[3], pos[4], pos[5]);
    return new Pose3d(position, rotation);
  }

  public Pose2d getTarget2D() {
    //Creates an array using the position values (0:2 are x,y,z, and 3:5 are yaw, roll, pitch. Not in those specific orders)
    double pos[] = m_botpose_target.getDoubleArray(new double[6]);
    Translation2d position2D = new Translation2d(pos[0], pos[1]);
    Rotation2d rotation2D = new Rotation2d(pos[3]);
    return new Pose2d(position2D, rotation2D);
  }

  //Gets only yaw, roll, and pitch using the same techniques as the previous method
  public Translation3d getTargetRotation() {
    double pos[] = m_botpose_target.getDoubleArray(new double[6]);
    Translation3d rotation = new Translation3d(pos[3], pos[4], pos[5]);
    return rotation;
  }

  //Methods to retun respective network table values
  public double getSimpleHorizontalAngle() {
    return m_tx.getDouble(0.0);
  }

  public double getSimpleVerticalAngle() {
    return m_ty.getDouble(0.0);
  }

  public double getNoteHorizontalAngle() {
    return m_tnx.getDouble(0.0);
  }

  public boolean hasNoteTarget() {
    return (m_tnv.getDouble(0.0) == 0.0) ? false : true;
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
