// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.CANBusIDs;
import static frc.robot.Constants.Drivetrain;
import static frc.robot.Constants.ChargeStation;

import frc.robot.RobotContainer;
import frc.robot.commands.Balance1ApproachCS;
import frc.robot.commands.Balance2DriveUpRamp;
import frc.robot.commands.Balance3DriveForwardNInches;
import frc.robot.commands.Balance4HoldPosition;

public class DrivetrainSubsystem extends PIDSubsystem {
  private final WPI_TalonFX leftFront;
  private final WPI_TalonFX rightFront;
  private final WPI_TalonFX leftRear;
  private final WPI_TalonFX rightRear;
  private final DifferentialDrive drive;

  private double lfEncPos;
  private double rfEncPos;
  private double lrEncPos;
  private double rrEncPos;
  private double avgEncoderPositionRaw;
  private double avgEncoderPositionInches;
  private double avgEncoderVelocityRaw;
  private double avgEncoderVelocityFeetPerSec;

  private PIDController controller;
  private static DrivetrainSubsystem instance;
  private double setpoint;

  public static DrivetrainSubsystem get() {
    if (instance == null) {
      PIDController ctlr = new PIDController(
          ChargeStation.kP, ChargeStation.kI, ChargeStation.kD);
      instance = new DrivetrainSubsystem(ctlr);
    }
    return instance;
  }

  /**
   * You probably want to call the static factory method get() instead of
   * creating a new instance of DrivetrainSubsystem...
   */
  public DrivetrainSubsystem(PIDController ctlr) {
    super(ctlr);
    controller = ctlr;
    setpoint = 0.0;

    // Default rotation direction is counter-clockwise
    // Left motors are inverted, right motors are not
    leftFront = new WPI_TalonFX(CANBusIDs.LEFT_FRONT_MOTOR);
    leftFront.setInverted(true);
    rightFront = new WPI_TalonFX(CANBusIDs.RIGHT_FRONT_MOTOR);
    rightFront.setInverted(false);
    leftRear = new WPI_TalonFX(CANBusIDs.LEFT_REAR_MOTOR);
    leftRear.setInverted(true);
    rightRear = new WPI_TalonFX(CANBusIDs.RIGHT_REAR_MOTOR);
    rightRear.setInverted(false);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    drive = new DifferentialDrive(leftFront, rightFront);
    disable();  // start in manual mode, not controlled by PID controller

    SmartDashboard.putNumber("LF Encoder Pos (Raw)", 0);
    SmartDashboard.putNumber("RF Encoder Pos (Raw)", 0);
    SmartDashboard.putNumber("LR Encoder Pos (Raw)", 0);
    SmartDashboard.putNumber("RR Encoder Pos (Raw)", 0);
    SmartDashboard.putNumber("Avg Encoder Pos (Raw)", 0);
    SmartDashboard.putNumber("LF Encoder Pos (Inches)", 0);
    SmartDashboard.putNumber("RF Encoder Pos (Inches)", 0);
    SmartDashboard.putNumber("LR Encoder Pos (Inches)", 0);
    SmartDashboard.putNumber("RR Encoder Pos (Inches)", 0);
    SmartDashboard.putNumber("Avg Encoder Pos (Inches)", 0);
    SmartDashboard.putNumber("Avg Encoder Speed (Raw)", 0);
    SmartDashboard.putNumber("Avg Encoder Speed (Inches)", 0);
    SmartDashboard.putData("Drivetrain Subsystem", this);

    setDefaultCommand(driveInTeleopModeCommand(this, RobotContainer.m_driverController));
  }

  public void updateEncoderData() {
    // This method will be called once per scheduler run
    lfEncPos = leftFront.getSelectedSensorPosition();
    rfEncPos = rightFront.getSelectedSensorPosition();
    lrEncPos = leftRear.getSelectedSensorPosition();
    rrEncPos = rightRear.getSelectedSensorPosition();
    avgEncoderPositionRaw = (lfEncPos + rfEncPos + lrEncPos + rrEncPos) / 4.0;

    /*
     * Talon FX integrated sensor has 2048 encoder units per shaft revolution
     * motor-to-wheel gear ratio = ?
     * wheel diameter = 6 inches
     * wheel circumference = 6*PI inches
     * Theoretical: pos_inches = pos_raw / 2048 / GearRatio * 6*PI = ___
     * Empirical: We measured the robot going ___ inches when the encoder
     *     reading increased by ___ -> pos_inches = pos_raw * __/__
     */
    double encoderUnitsPerInch = 10865.0;
    avgEncoderPositionInches = avgEncoderPositionRaw / encoderUnitsPerInch;

    avgEncoderVelocityRaw =
      (leftFront.getSelectedSensorVelocity() +
       rightFront.getSelectedSensorVelocity() +
       leftRear.getSelectedSensorVelocity() +
       rightRear.getSelectedSensorVelocity()) / 4.0;
    /*
     * Raw encoder velocity is given in "encoder units per 100 ms"
     * To convert to inches per second:
     *    raw encoder velocity (enc units / tenths-of-second) *
     *      (10 tenths-of-second/sec) *
     *      (1 inch / encoderUnitsPerInch enc units)
     *    = inches per second
     * raw encoder velocity * 10 / encoderUnitsPerInch = inches/sec
     * raw encoder velocity * 10 / encoderUnitsPerInch / 12 = feet/sec
     */
    avgEncoderVelocityFeetPerSec =
      avgEncoderVelocityRaw * 10.0 / encoderUnitsPerInch / 12.0;

    SmartDashboard.putNumber("LF Encoder Pos (Raw)", lfEncPos);
    SmartDashboard.putNumber("RF Encoder Pos (Raw)", rfEncPos);
    SmartDashboard.putNumber("LR Encoder Pos (Raw)", lrEncPos);
    SmartDashboard.putNumber("RR Encoder Pos (Raw)", rrEncPos);
    SmartDashboard.putNumber("Avg Encoder Pos (Raw)", avgEncoderPositionRaw);
    SmartDashboard.putNumber("Avg Encoder Pos (Inches)", Math.round(10.0*avgEncoderPositionInches)/10.0);
    SmartDashboard.putNumber("Avg Encoder Speed (Raw)", avgEncoderVelocityRaw);
    SmartDashboard.putNumber("Avg Encoder Speed (ft/sec)", Math.round(100.0*avgEncoderVelocityFeetPerSec)/100.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateEncoderData();
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
      .andThen(new Balance3DriveForwardNInches(drive, ChargeStation.INCHES_PAST_LEVELING))
      .andThen(new Balance4HoldPosition(drive));
    return cmd;
  }

  /**
   * Factory method to produce a Command for driving in tele-operated mode.
   *
   * @return (CommandBase) command for driving based on user input via the
   * joysticks.

   */
  public Command driveInTeleopModeCommand(DrivetrainSubsystem drive, CommandXboxController stick) {
    return new InstantCommand(() -> driveWithJoysticks(stick), drive);
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

  public double avgEncoderPositionInches() {
    return avgEncoderPositionInches;
  }

  public void setCoasting(boolean coasting) {
    NeutralMode mode;
    if(coasting) {
      mode = NeutralMode.Coast;
    } else {
      mode = NeutralMode.Brake;
    }
    leftFront.setNeutralMode(mode);
    rightFront.setNeutralMode(mode);
    leftRear.setNeutralMode(mode);
    rightRear.setNeutralMode(mode);
  }

  public void stop() {
    drive.stopMotor();
  }

  /**
   * Method required for PID control as a PIDSubsystem subclass
   */
  public void useOutput(double value, double setpoint) {
    driveStraightAtFixedRate(value);
  }

  /**
   * Method required for PID control as a PIDSubsystem subclass
   */
  public double getMeasurement() {
    return avgEncoderPositionInches();
  }
}
