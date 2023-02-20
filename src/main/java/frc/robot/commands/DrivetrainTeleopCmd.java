package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainTeleopCmd extends CommandBase {
  public DrivetrainSubsystem drivetrain;

  public DrivetrainTeleopCmd(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.driveWithJoysticks(RobotContainer.m_driverController);
  }

}
