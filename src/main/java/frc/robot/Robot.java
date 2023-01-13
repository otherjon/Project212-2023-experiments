// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public final File deployDir = Filesystem.getDeployDirectory();
  private String gitCommitHash;
  private String gitBranch;
  private String gitTags;

  public String getGitCommitHash() {
    if (gitCommitHash == null) {
      File commitFile = new File(deployDir, "commit.txt");
      try {
        BufferedReader br = new BufferedReader(new FileReader(commitFile));
        gitCommitHash = br.readLine().trim();
        br.close();
      } catch (IOException e) {
        gitCommitHash = "[error reading commit.txt]";
      }
    }
    return gitCommitHash;
  }

  public String getGitBranch() {
    if (gitBranch == null) {
      File branchFile = new File(deployDir, "branch.txt");
      try {
        BufferedReader br = new BufferedReader(new FileReader(branchFile));
        gitBranch = br.readLine().trim();
        br.close();
      } catch (IOException e) {
        gitBranch = "[error reading branch.txt]";
      }
    }
    return gitBranch;
  }

  public String getGitTags() {
    if (gitTags == null) {
      File tagsFile = new File(deployDir, "tags.txt");
      try {
        BufferedReader br = new BufferedReader(new FileReader(tagsFile));
        gitTags = br.readLine().trim();
        br.close();
      } catch (IOException e) {
        gitTags = "[error reading tags.txt]";
      }
    }
    return gitTags;
  }

  public void logGitInfo() {
    String gCommit = getGitCommitHash();
    String gBranch = getGitBranch();
    String gTags = getGitTags();
    DataLogManager.log("****************************************");
    DataLogManager.log("Software version: git commit hash [" + gCommit +
                       "] (branch " + gBranch + ")");
    if (gTags.equals("")) {
      DataLogManager.log("  (untagged)");
    } else {
      DataLogManager.log("  tagged version: [" + gTags + "]");
    }
    DataLogManager.log("****************************************");
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    logGitInfo();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
