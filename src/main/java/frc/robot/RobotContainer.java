// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.GitInfoSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final GitInfoSubsystem m_gitInfo = new GitInfoSubsystem();
  private final NavXSubsystem m_navx = new NavXSubsystem();
  private final DrivetrainSubsystem m_drive = DrivetrainSubsystem.get();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Command instances
  private final Command autoCmd = m_drive.balanceOnChargingStationCommand(m_drive, m_navx);
  private final Command teleopCmd = m_drive.driveInTeleopModeCommand(m_drive, m_driverController);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();  // bind triggers (controller button presses) to commands
    logGitInfo();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Cancel all commands in progress when the Xbox controller's X button is pressed
    m_driverController.x().onTrue(new InstantCommand(

        () -> CommandScheduler.getInstance().cancelAll()));
    // Run the charging station auto-balance procedure when the Xbox controller's A button is pressed
    m_driverController.a().onTrue(
        m_drive.balanceOnChargingStationCommand(m_drive, m_navx));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCmd;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopCmd;
  }

  /**
   * Log information from the GitInfoSubsystem to the console.
   */
  public void logGitInfo() {
    String gCommit = m_gitInfo.commitHash();
    String gBranch = m_gitInfo.branch();
    String gTags = m_gitInfo.tags();
    String gMods = m_gitInfo.modifiedFiles();
    if (gMods == "") {
      // No modified files on the robot computer -- good!
      DataLogManager.log("========================================");
      DataLogManager.log("Software version: git commit hash [" + gCommit +
                         "] (branch " + gBranch + ")");
      if (gTags.equals("")) {
        DataLogManager.log("  (untagged)");
      } else {
        DataLogManager.log("  tagged version: [" + gTags + "]");
      }
      DataLogManager.log("========================================");
    } else {
      // Uh-oh, some files have been modified locally!
      // This means that tags and commit hashes are invalid.
      // And if this isn't a short-term experiment or a competition emergency,
      //   then someone on the software team is being very naughty!
      DataLogManager.log("=========!!==!!==!!==!!==!!==!!=========");
      DataLogManager.log("Software version: LOCALLY MODIFIED CODE");
      DataLogManager.log("  originally based off branch " + gBranch +
                         ", commit " + gCommit);
      DataLogManager.log("=========!!==!!==!!==!!==!!==!!=========");
    }
  }
}
