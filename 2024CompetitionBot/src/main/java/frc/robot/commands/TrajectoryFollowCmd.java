// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSub;

public class TrajectoryFollowCmd extends Command {
  /** Creates a new TrajectoryFollowCmd. */
  // Reference: https://github.com/wpilibsuite/allwpilib/blob/435bbb6a8ccde148580d3e7d84d41d8f8446e03b/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java

  private final DrivetrainSub m_drivetrainSub;


  public TrajectoryFollowCmd(DrivetrainSub drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void TrajectoryFollow() { // Using trajectory library
    Rotation2d currentRotation = m_drivetrainSub.getRotation();
    // Example path
    List<Pose2d> points = new ArrayList<Pose2d>();
    points
        .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(), currentRotation));
    points
        .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 1.0 + m_drivetrainSub.getPos().getY(), currentRotation));
    points
        .add(new Pose2d(1.0 + m_drivetrainSub.getPos().getX(), 1.0 + m_drivetrainSub.getPos().getY(), currentRotation));
    points
        .add(new Pose2d(1.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(), currentRotation));
    points
        .add(new Pose2d(0.0 + m_drivetrainSub.getPos().getX(), 0.0 + m_drivetrainSub.getPos().getY(), currentRotation));


    //SwerveDriveKinematicsConstraint kinematicsConstraint =
    //     new SwerveDriveKinematicsConstraint(m_kinematics, kMaxDriveSpeed); // Makes sure the trajectory isn't calculated above max speed
    TrajectoryConfig tConfig =
        new TrajectoryConfig(DrivetrainSub.kMaxDriveSpeed, 10.0).setKinematics(m_drivetrainSub.m_kinematics);

    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(points, tConfig); // In meters
    var thetaController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)); // TODO: Get real values for this
    thetaController.enableContinuousInput(-180.0, 180.0); // TODO: Find out if this should be continuous
    SwerveControllerCommand swerveCommand = new SwerveControllerCommand(testTrajectory,
        m_drivetrainSub::getOdometryPose2d,
        m_drivetrainSub.m_kinematics, m_drivetrainSub.m_odometryPIDx, m_drivetrainSub.m_odometryPIDy, thetaController,
        m_drivetrainSub::driveStates, m_drivetrainSub);
  }
}
