// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class NavXSubsystem extends SubsystemBase {

  private AHRS navx = new AHRS();
  private double accel_x;
  private double accel_y;
  private double accel_z;
  private double yaw;
  private double pitch;
  private double roll;

  /** Creates a new NavXSubsystem. */
  public NavXSubsystem() {}

  /**
   * Command to run NavX calibration routine.  Not required at startup, since
   * the NavX calibrates automatically at startup.  Calibration takes roughly
   * 5 seconds of real time.
   *
   * @return (CommandBase) a command to run calibration
   */
  public CommandBase calibrateCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> { navx.calibrate(); });
  }

  /**
   * Subsystem condition method to determine whether the NavX calibration has
   * completed.
   *
   * @return (boolean) true if and only if the sensor calibration has completed
   * and the sensor is no longer still calibrating
   */
  public boolean isCalibrated() {
    return ! navx.isCalibrating();
  }

  /**
   * Subsystem condition method to determine whether the NavX is connected and
   * passes basic internal functionality tests.
   *
   * @return (boolean) true if and only if the NavX appears to be working
   */
  public boolean isConnected() {
    return navx.isConnected();
  }

  public double getYaw()   { return yaw; }
  public double getPitch() { return pitch; }
  public double getRoll()  { return roll; }

  private void updateSensorVariables() {
    // Update private variables using data from the sensor
    accel_x = navx.getWorldLinearAccelX();
    accel_y = navx.getWorldLinearAccelY();
    accel_z = navx.getWorldLinearAccelZ();
    yaw = navx.getYaw();
    pitch = navx.getPitch();
    roll = navx.getRoll();
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("X Accel", accel_x);
    SmartDashboard.putNumber("Y Accel", accel_y);
    SmartDashboard.putNumber("Z Accel", accel_z);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Roll", roll);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSensorVariables();
    updateDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
