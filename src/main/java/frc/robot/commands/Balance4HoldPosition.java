// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** Balance on the Charging Station (CS): Part 4
 *
 *  The task of balancing on the charging station requires multiple commands
 *  to be executed serially.  This command represents Part 4, actively holding
 *  our position on the leveled platform, resisting gravity when other robots
 *  tilt the platform to get on.  Part 4 does not end until either the driver
 *  presses the appropriate button on the controller, or the robot changes
 *  modes (e.g. auto-to-teleop, enabled-to-disabled).
 */
public class Balance4HoldPosition extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drive;
  private final double encoderInitialReadingInches;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   */
  public Balance4HoldPosition(DrivetrainSubsystem drive) {
    super(drive.getController(),
          drive::getMeasurement,
          () -> drive.getMeasurement(),
          val -> drive.useOutput(val, drive.getMeasurement()),
          drive);
    m_drive = drive;
    addRequirements(drive);    // declare subsystem dependencies
    encoderInitialReadingInches = m_drive.avgEncoderPositionInches();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_drive.setCoasting(false);
    m_drive.setSetpoint(m_drive.getMeasurement());
    m_drive.enable();  // turn on PID control
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drive.setCoasting(true);
    m_drive.disable();  // turn off PID control
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // hold position forever, until either we hit the controller's cancel
    // button, or we change modes (auto-to-teleop, disable the robot, etc.)
    return false;
  }
}
