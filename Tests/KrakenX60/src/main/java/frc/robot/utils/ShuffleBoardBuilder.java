// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.tests.RunTestsGrp;

/** Add your docs here. */
public class ShuffleBoardBuilder {
  public int m_nextTestRow = 0;
  private GenericEntry m_testOverallResult;

  public ShuffleBoardBuilder(RunTestsGrp testsCommand) {
    ShuffleboardTab tab;
    ShuffleboardLayout layout;

    // Establish all of the Shuffleboard tabs we'll want to use (in order of appearance)
    tab = Shuffleboard.getTab("Tests");
    m_testOverallResult = tab.add("Overall", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(0, 0)
        .getEntry();
    tab.add("Run Tests", testsCommand);
    m_nextTestRow++;


    tab = Shuffleboard.getTab("About");
    layout = tab.getLayout("Build Info", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0);
    // WARNING: These "GVersion" values get created when you first build this project
    // It's okay for them to be red until then.  See build.gradle and utils/GVersion.java.
    layout.add("Project", GVersion.MAVEN_NAME);
    layout.add("Date", GVersion.BUILD_DATE);
    layout.add("Git Dirty?", GVersion.DIRTY); // 1 = Code contains changes not committed to Git, 0 = clean, -1 = problem
    layout.add("Git Branch", GVersion.GIT_BRANCH);
    layout.add("Git Commit Date", GVersion.GIT_DATE); // Doesn't mean much unless dirty = 0
    layout.add("Git Commit SHA", GVersion.GIT_SHA); // Doesn't mean much unless dirty = 0
  }

  /**
   * Sets the overall result "light"
   * 
   * @param result true = green (pass_, false = red (failed or incomplete)
   */
  public void setTestsOverallResult(boolean result) {
    m_testOverallResult.setBoolean(result);
  }
}
