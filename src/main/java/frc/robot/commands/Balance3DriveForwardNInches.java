// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Balance on the Charging Station (CS): Part 3
 *
 *  The task of balancing on the charging station requires multiple commands
 *  to be executed serially.  This command represents Part 3, driving forward
 *  on the leveled platform to gain a bit of safety margin.  Part 3 is complete
 *  when the drivetrain's average encoder reading indicates that we've reached
 *  our destination.
 */
public class Balance3DriveForwardNInches extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public static final double DRIVE_SPEED = 0.2;
  private final DrivetrainSubsystem m_drive;
  private double encoderInitialReadingInches;
  private double inchesForward;
  private double encoderTargetReadingInches;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   * @param navx The NavXSubsystem used by this command.
   */
  public Balance3DriveForwardNInches(DrivetrainSubsystem drive, double inches) {
    m_drive = drive;
    inchesForward = inches;
    addRequirements(drive);    // declare subsystem dependencies
    DataLogManager.log("=== [DEBUG] instantiated Balance3DriveForwardNInches command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderInitialReadingInches = m_drive.avgEncoderPositionInches();
    encoderTargetReadingInches = encoderInitialReadingInches + inchesForward;
    DataLogManager.log("=== [DEBUG] initialized Balance3DriveForwardNInches command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraightAtFixedRate(DRIVE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("=== [DEBUG] ended Balance3DriveForwardNInches command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished when the encoders indicate we've arrived
    return m_drive.avgEncoderPositionInches() >= encoderTargetReadingInches;
  }
}