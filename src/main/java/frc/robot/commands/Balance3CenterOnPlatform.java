// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Balance on the Charging Station (CS): Part 3
 *
 *  The task of balancing on the charging station requires multiple commands
 *  to be executed serially.  This command represents Part 3, driving forward
 *  on the leveled platform to gain a bit of safety margin.  Part 3 is complete
 *  when the Timer indicates that we've been driving forward at low speed for
 *  a fixed time.
 *
 *  In the future, when we have a drivetrain with Talon FXes and Falcon 500s,
 *  we'll have built-in encoders.  Encoders will allow us to end this command
 *  when we've travelled a certain distance, rather than a certain time.
 *  However, since this is being tested on Rockstar and Rockstar doesn't have
 *  (working) encoders, we'll try the "drive blindly forward for a fixed time"
 *  approach.
 */
public class Balance3CenterOnPlatform extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public static final double DRIVE_TIME = 2.0;
  public static final double DRIVE_SPEED = 0.2;
  private final DrivetrainSubsystem m_drive;
  private Timer m_timer;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   */
  public Balance3CenterOnPlatform(DrivetrainSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);    // declare subsystem dependencies
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraightAtFixedRate(DRIVE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finished when the elapsed time reaches the threshold
    return m_timer.get() >= DRIVE_TIME;
  }
}
