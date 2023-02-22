// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for a binary sensor.
 *
 * A binary sensor has two values: logical true, usually +5V, and logical false,
 * i.e. 0V.  These sensors are wired to the roboRIO's digital I/O (DIO) ports
 * as inputs.  Examples include limit switches and magnetic sensors such as
 * Hall-effect sensors.
 */

public class BinarySensorSubsystem extends SubsystemBase {
  public DigitalInput sensor;
  public boolean sensorValue;

  /** Creates a new BinarySensorSubsystem. */
  public BinarySensorSubsystem(int dio_port) {
    sensor = new DigitalInput(dio_port);
    sensorValue = sensor.get();
    SmartDashboard.putBoolean("Sensor value", sensorValue);
  }

  @Override
  public void periodic() {
    sensorValue = sensor.get();
    SmartDashboard.putBoolean("Sensor value", sensorValue);
  }
}
