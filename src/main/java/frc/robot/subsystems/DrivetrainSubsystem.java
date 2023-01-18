// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.CANBusIDs;
import static frc.robot.Constants.Drivetrain;
import frc.robot.commands.Balance1ApproachCS;
import frc.robot.commands.Balance2DriveUpRamp;
import frc.robot.commands.Balance3CenterOnPlatform;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private final WPI_TalonSRX leftFront;
  private final WPI_TalonSRX rightFront;
  private final WPI_TalonSRX leftRear;
  private final WPI_TalonSRX rightRear;

  private final DifferentialDrive drive;

  public DrivetrainSubsystem() {
    leftFront = new WPI_TalonSRX(CANBusIDs.LEFT_FRONT_MOTOR);
    leftFront.setInverted(true);
    rightFront = new WPI_TalonSRX(CANBusIDs.RIGHT_FRONT_MOTOR);
    rightFront.setInverted(false);
    leftRear = new WPI_TalonSRX(CANBusIDs.LEFT_REAR_MOTOR);
    leftRear.setInverted(true);
    rightRear = new WPI_TalonSRX(CANBusIDs.RIGHT_REAR_MOTOR);
    rightRear.setInverted(false);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    drive = new DifferentialDrive(leftFront, rightFront);

    SmartDashboard.putNumber("Drivetrain Distance (Feet)", 0);
    SmartDashboard.putNumber("Left Encoder Distance (Inches)", 0);
    SmartDashboard.putNumber("Right Encoder Distance (Inches)", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Factory method to produce a SequentialCommandGroup to balance on the
   * charging station.
   *
   * @return (SequentialCommandGroup) command group for balancing on the
   * charging station
   */
  public SequentialCommandGroup balanceOnChargingStationCommand(
      DrivetrainSubsystem drive, NavXSubsystem navx) {
    SequentialCommandGroup cmd = new Balance1ApproachCS(drive, navx)
      .andThen(new Balance2DriveUpRamp(drive, navx))
      .andThen(new Balance3CenterOnPlatform(drive, navx));
    return cmd;
  }

  /**
   * Factory method to produce a Command for driving in tele-operated mode.
   *
   * @return (CommandBase) command for driving based on user input via the
   * joysticks.

   */
  public Command driveInTeleopModeCommand(DrivetrainSubsystem drive, NavXSubsystem navx, CommandXboxController stick) {
    // TODO: add requirements to command!
    return new InstantCommand(() -> driveWithJoysticks(stick));
  }

  public void driveWithJoysticks(CommandXboxController stick) {
    double maxFwd = Drivetrain.MAX_FORWARD_SPEED;
    double maxTurn = Drivetrain.MAX_TURN_SPEED;
    // right trigger is forward, left trigger is reverse
    double rightTriggerAmount = stick.getRightTriggerAxis();
    double leftTriggerAmount = stick.getLeftTriggerAxis();
    double turnAmount = stick.getRightX();

    double forward = maxFwd * (rightTriggerAmount - leftTriggerAmount);
    double turn = maxTurn * turnAmount;
    drive.arcadeDrive(forward, turn);
  }

  public void driveStraightAtFixedRate(double rate) {
    drive.arcadeDrive(rate, 0.0);
  }

  public void stop() {
    drive.stopMotor();
  }

}
