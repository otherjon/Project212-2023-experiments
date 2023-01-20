// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXSubsystem;

/** Balance on the Charging Station (CS): Part 1
 *
 *  The task of balancing on the charging station requires multiple commands
 *  to be executed serially.  This command represents Part 1, approaching the
 *  charging station.  Part 1 is complete when the gyroscope subsystem
 *  indicates that we are on the charging station's ramp.
 */
public class Balance1ApproachCS extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drive;
  private final NavXSubsystem m_navx;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   * @param navx The NavXSubsystem used by this command.
   */
  public Balance1ApproachCS(DrivetrainSubsystem drive, NavXSubsystem navx) {
    m_drive = drive;
    m_navx = navx;
    addRequirements(drive, navx);    // declare subsystem dependencies
    DataLogManager.log("=== [DEBUG] instantiated Balance1ApproachCS command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("=== [DEBUG] initialized Balance1ApproachCS command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraightAtFixedRate(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("=== [DEBUG] ended Balance1ApproachCS command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished when we're tilted backward more than 5 degrees
    return m_navx.getPitch() < -5.0;
  }
}
