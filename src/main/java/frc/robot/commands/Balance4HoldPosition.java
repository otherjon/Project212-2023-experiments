// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import static frc.robot.Constants.ChargeStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;

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
  private final PIDController controller;

  /**
   * Creates a new command.
   *
   * @param drive The DriveSubsystem used by this command.
   */
  public Balance4HoldPosition(DrivetrainSubsystem drive) {
    // TODO: rewrite to match PIDCommand ctor signature
    super(drive.getController(),
          drive::getMeasurement,
          () -> drive.getMeasurement(),
          val -> drive.useOutput(val),
          drive);
    m_drive = drive;
    addRequirements(drive);    // declare subsystem dependencies
    encoderInitialReadingInches = m_drive.avgEncoderPositionInches();
    controller = PIDController(ChargeStation.kP, ChargeStation.kI, ChargeStation.kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setCoasting(false);
    m_drive.enable();  // turn on PID control
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraightAtFixedRate(DRIVE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
