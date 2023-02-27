// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SmartDashboard;

import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ColorSensorSubsystem extends SubsystemBase {
  /**
   * Wrapper for color sensors based on the TMD3782 chip, such as the
   * REV Robotics Color Sensor V2 (but *not* the V3!).
   *
   * Chip reference: http://www.revrobotics.com/content/docs/TMD3782_v2.pdf
   *
   * This code based on examples at
   *  https://www.chiefdelphi.com/t/writing-code-for-a-color-sensor/167303/4
   *
   */

  // See TMD3782 data sheet, pages 20-27
  protected final static int CMD = 0x80;
  protected final static int MULTI_BYTE_BIT = 0x20;
  protected final static int ENABLE_REGISTER  = 0x00;
  protected final static int ATIME_REGISTER   = 0x01; // integration time
  protected final static int CLEAR_CH_INTERRUPT_THRESHOLDS_REGISTERS = 0x04;
  protected final static int PROX_INTERRUPT_THRESHOLDS_REGISTERS = 0x08;
  protected final static int INTERRUPT_PERSISTENCE_REGISTER = 0x0C;
  protected final static int CONFIG_REGISTER   = 0x0D;
  protected final static int PPULSE_REGISTER   = 0x0E;
  protected final static int CONTROL_REGISTER  = 0x0F;
  protected final static int REVISION_REGISTER = 0x11;
  protected final static int ID_REGISTER       = 0x12;
  protected final static int STATUS_REGISTER   = 0x13;
  protected final static int CDATA_REGISTER    = 0x14;
  protected final static int RDATA_REGISTER    = 0x16;
  protected final static int GDATA_REGISTER    = 0x18;
  protected final static int BDATA_REGISTER    = 0x1A;
  protected final static int PDATA_REGISTER    = 0x1C;

  // See TMD3782 data sheet, page 20
  protected final static int PON   = 0b00000001;  // power-on
  protected final static int AEN   = 0b00000010;  // ADC enable
  protected final static int PEN   = 0b00000100;  // prox sensor enable
  protected final static int WEN   = 0b00001000;  // wait-state enable
  protected final static int AIEN  = 0b00010000;  // enable interrupt for color
  protected final static int PIEN  = 0b00100000;  // enable interrupt for prox

  private final double integrationTime = 10;

  private I2C sensor;
  private ByteBuffer buffy = ByteBuffer.allocate(8);
  public short red = 0, green = 0, blue = 0, prox = 0;

  public ColorSensorSubsystem(I2C.Port port) {
    buffy.order(ByteOrder.LITTLE_ENDIAN);
    sensor = new I2C(port, 0x39);  // 0x39 = I2C address of REV ColorSensor V2

    // Turn the sensor on, enable the ADC and prox sensor
    sensor.write(CMD | 0x00, PON | AEN | PEN);

    // configure the integration time (duration to sample color data)
    // see TMD3782 data sheet p.21 for the formula
    sensor.write(CMD | ATIME_REGISTER, (int) (256 - integrationTime/2.38));

    // set proximity pulse count to max
    sensor.write(CMD | PPULSE_REGISTER, 0b1111);
  }

  public static short parsePositiveShortFromBuffer(ByteBuffer buf, int index) {
    short val = buf.getShort(index);
    if (val < 0) {
      // make a positive short, the hard way
      val += 0b10000000000000000;
    }
    return val;
  }

  public void updateSensorData() {
    buffy.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | RDATA_REGISTER, 8, buffy);
    red = parsePositiveShortFromBuffer(buffy, 0);
    green = parsePositiveShortFromBuffer(buffy, 2);
    blue = parsePositiveShortFromBuffer(buffy, 4);
    prox = parsePositiveShortFromBuffer(buffy, 6);
  }

  public int status() {
    buffy.clear();
    sensor.read(CMD | STATUS_REGISTER, 1, buffy);
    return buffy.get(0);
  }

  public void free() {
    sensor.free();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Status", status());
    SmartDashboard.putNumber("Red", red);
    SmartDashboard.putNumber("Green", green);
    SmartDashboard.putNumber("Blue", blue);
    SmartDashboard.putNumber("Prox", prox);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSensorData();
    updateDashboard();
  }
}
