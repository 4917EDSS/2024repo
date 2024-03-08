// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Handler;
import java.util.logging.LogManager;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Logger m_logger = Logger.getLogger(Robot.class.getName());
  //power distribution board
  private static final int PDH_CAN_ID = 1;

  // private static final int NUM_PDH_CHANNELS =24;

  PowerDistribution m_pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean m_isInitialized = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Setup the logger
    Logger rootLogger = LogManager.getLogManager().getLogger("");
    rootLogger.setLevel(Constants.kLogLevel);
    for(Handler handler : rootLogger.getHandlers()) {
      handler.setLevel(Constants.kLogLevel);
    }

    // Log a message to each level to show which levels are currently enabled
    m_logger.severe("Severe log level enabled");
    m_logger.warning("Warning log level enabled");
    m_logger.info("Info log level enabled");
    m_logger.config("Config log level enabled");
    m_logger.fine("Fine log level enabled");
    m_logger.finer("Finer log level enabled");
    m_logger.finest("Finest log level enabled");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //TODO: Resolved - Remove this
    // SmartDashboard.putNumber("Voltage0", m_pdh.getCurrent(0));
    // SmartDashboard.putNumber("Voltage1", m_pdh.getCurrent(1));
    // SmartDashboard.putNumber("Voltage2", m_pdh.getCurrent(2));
    // SmartDashboard.putNumber("Voltage3", m_pdh.getCurrent(3));
    // SmartDashboard.putNumber("Voltage4", m_pdh.getCurrent(4));
    // SmartDashboard.putNumber("Voltage5", m_pdh.getCurrent(5));
    // SmartDashboard.putNumber("Voltage6", m_pdh.getCurrent(6));
    // SmartDashboard.putNumber("Voltage7", m_pdh.getCurrent(7));
    // SmartDashboard.putNumber("Voltage8", m_pdh.getCurrent(8));
    // SmartDashboard.putNumber("Voltage9", m_pdh.getCurrent(9));
    // SmartDashboard.putNumber("Voltage10", m_pdh.getCurrent(10));
    // SmartDashboard.putNumber("Voltage11", m_pdh.getCurrent(11));
    // SmartDashboard.putNumber("Voltage12", m_pdh.getCurrent(12));
    // SmartDashboard.putNumber("Voltage13", m_pdh.getCurrent(13));
    // SmartDashboard.putNumber("Voltage14", m_pdh.getCurrent(14));
    // SmartDashboard.putNumber("Voltage15", m_pdh.getCurrent(15));
    // SmartDashboard.putNumber("Voltage16", m_pdh.getCurrent(16));
    // SmartDashboard.putNumber("Voltage17", m_pdh.getCurrent(17));
    // SmartDashboard.putNumber("Voltage18", m_pdh.getCurrent(18));
    // SmartDashboard.putNumber("Voltage19", m_pdh.getCurrent(19));
    // SmartDashboard.putNumber("Voltage20", m_pdh.getCurrent(20));
    // SmartDashboard.putNumber("Voltage21", m_pdh.getCurrent(21));
    // SmartDashboard.putNumber("Voltage22", m_pdh.getCurrent(22));
    // SmartDashboard.putNumber("Voltage23", m_pdh.getCurrent(23));
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if(m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Reset the subsystems if this is the first time we run or if we have signaled that we should reset
    if(!m_isInitialized) {
      m_robotContainer.initSubsystems();
      m_isInitialized = true;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if(m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Reset the subsystems if this is the first time we run or if we have signaled that we should reset
    // TODO Restore this for competition
    if(true /* !m_isInitialized */) {
      m_robotContainer.initSubsystems();
      m_isInitialized = true;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Reset most parts of the robot
    m_logger.warning("Resetting robot subsystems via testInit");
    m_isInitialized = false;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
