// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.Constants;

/** Balance on the Charging Station (CS): Part 2
 *
 *  The task of balancing on the charging station requires multiple commands
 *  to be executed serially.  This command represents Part 2, driving up the
 *  charging station's ramp.  Part 2 is complete when the gyroscope subsystem
 *  indicates that we are far enough up the ramp and onto the platform that
 *  the charging station has leveled itself.
 */
public class Balance2DriveUpRamp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drive;
  private final NavXSubsystem m_navx;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   * @param navx The NavXSubsystem used by this command.
   */
  public Balance2DriveUpRamp(DrivetrainSubsystem drive, NavXSubsystem navx) {
    m_drive = drive;
    m_navx = navx;
    addRequirements(drive, navx);    // declare subsystem dependencies
    DataLogManager.log("=== [DEBUG] instantiated Balance2DriveUpRamp command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("=== [DEBUG] initialized Balance2DriveUpRamp command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraightAtFixedRate(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("=== [DEBUG] pitch is now " + String.valueOf(m_navx.getPitch()));
    DataLogManager.log("=== [DEBUG] ended Balance2DriveUpRamp command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished when we're minimally tilted front-to-back (within tolerance)
    double pitch = m_navx.getPitch();
    double nearly_level = Constants.ChargeStation.RAMP_LEVEL_PITCH_DEGREES;
    return (pitch < nearly_level && pitch > -nearly_level);
  }
}
