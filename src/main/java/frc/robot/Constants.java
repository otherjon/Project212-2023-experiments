// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class CANBusIDs {
    public static final int LEFT_TOP_MOTOR  = 1;
    public static final int RIGHT_TOP_MOTOR = 12;
    public static final int LEFT_BOTTOM_MOTOR   = 0;
    public static final int RIGHT_BOTTOM_MOTOR  = 15;
  }
  public static class Drivetrain {
    public static final double MAX_FORWARD_SPEED = 0.8;
    public static final double MAX_TURN_SPEED = 0.8;
  }
  public static class ChargeStation {
    // constants relating to the task of balancing on the charging station
    public static final double INCHES_PAST_LEVELING = 1.0;
    public static final double RAMP_PITCH_DEGREES = 5.0;
    public static final double RAMP_LEVEL_PITCH_DEGREES = 3.0;
    public static final double DRIVE_SPEED = 0.3;
    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}