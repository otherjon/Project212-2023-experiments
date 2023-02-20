// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.CANBusIDs;
import static frc.robot.Constants.Drivetrain;
import static frc.robot.Constants.ChargeStation;
import frc.robot.commands.Balance1ApproachCS;
import frc.robot.commands.Balance2DriveUpRamp;
import frc.robot.commands.Balance3DriveForwardNInches;
import frc.robot.commands.Balance4HoldPosition;

public class DrivetrainSubsystem extends PIDSubsystem {
  /*
   * Left- and right-side gearboxes each are driven by two motors, at the top
   * and bottom of the gearboxes.  Power is transferred to wheels by chains
   * from there.  But this is why the motors are "top" and "bottom".
   */
  private final WPI_TalonFX leftTop;
  private final WPI_TalonFX rightTop;
  private final WPI_TalonFX leftBottom;
  private final WPI_TalonFX rightBottom;
  private final DifferentialDrive drive;

  private double ltEncPos;
  private double rtEncPos;
  private double lbEncPos;
  private double rbEncPos;
  private double avgEncoderPositionRaw;
  private double avgEncoderPositionInches;
  private double avgEncoderVelocityRaw;
  private double avgEncoderVelocityFeetPerSec;

  private PIDController controller;
  private static DrivetrainSubsystem instance;
  private double setpoint;

  /*
   * Talon FX integrated sensor has 2048 encoder units per shaft revolution
   * motor-to-wheel gear ratio = ?
   * wheel diameter = 6 inches
   * wheel circumference = 6*PI inches
   * Theoretical: pos_inches = pos_raw / 2048 / GearRatio * 6*PI = ___
   * Empirical: We measured the robot going ___ inches when the encoder
   *     reading increased by ___ -> pos_inches = pos_raw * __/__
   */
  double ENCODER_UNITS_PER_INCH = 1000.0;

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
    leftTop = new WPI_TalonFX(CANBusIDs.LEFT_TOP_MOTOR);
    leftTop.setInverted(true);
    rightTop = new WPI_TalonFX(CANBusIDs.RIGHT_TOP_MOTOR);
    rightTop.setInverted(false);
    leftBottom = new WPI_TalonFX(CANBusIDs.LEFT_BOTTOM_MOTOR);
    leftBottom.setInverted(true);
    rightBottom = new WPI_TalonFX(CANBusIDs.RIGHT_BOTTOM_MOTOR);
    rightBottom.setInverted(false);

    leftBottom.follow(leftTop);
    rightBottom.follow(rightTop);

    drive = new DifferentialDrive(leftTop, rightTop);
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
  }

  public void updateEncoderData() {
    // This method will be called once per scheduler run
    ltEncPos = leftTop.getSelectedSensorPosition();
    rtEncPos = rightTop.getSelectedSensorPosition();
    lbEncPos = leftBottom.getSelectedSensorPosition();
    rbEncPos = rightBottom.getSelectedSensorPosition();
    avgEncoderPositionRaw = (ltEncPos + rtEncPos + lbEncPos + rbEncPos) / 4.0;

    avgEncoderPositionInches = avgEncoderPositionRaw / ENCODER_UNITS_PER_INCH;

    avgEncoderVelocityRaw =
      (leftTop.getSelectedSensorVelocity() +
       rightTop.getSelectedSensorVelocity() +
       leftBottom.getSelectedSensorVelocity() +
       rightBottom.getSelectedSensorVelocity()) / 4.0;
    /*
     * Raw encoder velocity is given in "encoder units per 100 ms"
     * To convert to inches per second:
     *    raw encoder velocity (enc units / tenths-of-second) *
     *      (10 tenths-of-second/sec) *
     *      (1 inch / ENCODER_UNITS_PER_INCH enc units)
     *    = inches per second
     * raw encoder velocity * 10 / ENCODER_UNITS_PER_INCH = inches/sec
     * raw encoder velocity * 10 / ENCODER_UNITS_PER_INCH / 12 = feet/sec
     */
    avgEncoderVelocityFeetPerSec =
      avgEncoderVelocityRaw * 10.0 / ENCODER_UNITS_PER_INCH / 12.0;

    SmartDashboard.putNumber("LF Encoder Pos (Raw)", ltEncPos);
    SmartDashboard.putNumber("RF Encoder Pos (Raw)", rtEncPos);
    SmartDashboard.putNumber("LR Encoder Pos (Raw)", lbEncPos);
    SmartDashboard.putNumber("RR Encoder Pos (Raw)", rbEncPos);
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

  public double inchesToEncoderUnits(double inches) {
    return inches * ENCODER_UNITS_PER_INCH;
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
    DataLogManager.log("=== [DEBUG] teleop command invoked");
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
    leftTop.setNeutralMode(mode);
    rightTop.setNeutralMode(mode);
    leftBottom.setNeutralMode(mode);
    rightBottom.setNeutralMode(mode);
  }

  public void stop() {
    drive.stopMotor();
  }

  /**
   * Method required for PID control as a PIDSubsystem subclass
   */
  public void useOutput(double value, double setpoint) {
    // TODO: Make this method return to the position setpoint
    // TODO: Make part 4 of the balancing command enable PID control
    driveStraightAtFixedRate(value);
  }

  /**
   * Method required for PID control as a PIDSubsystem subclass
   */
  public double getMeasurement() {
    return avgEncoderPositionInches();
  }
}
