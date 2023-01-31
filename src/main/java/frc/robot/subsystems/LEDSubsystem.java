// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Blinkin is controlled via PWM similarly to a Spark motor controller.
 * In fact, the output is *so* similar that there's no difference at all!
 *
 * The Blinkin supports 100 patterns, which can be selected by setting the
 * PWM output to -0.99, -0.97, -0.95, ... 0.95, 0.97, 0.99
 * (in increments of 0.02)
 *
 * See pages 14-17 of the Blinkin user manual:
 * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
 */

public class LEDSubsystem extends SubsystemBase {
  /**
   * The Blinkin LED driver is available for sale from REV Robotics:
   * https://www.revrobotics.com/rev-11-1105/
   *
   * Blinkin user manual (linked from the page above):
   * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  class Blinkin extends Spark {
    public Blinkin(int pwmPort) { super(pwmPort); }
  }
  
  public Blinkin ledController;
  public double pwmSetting; // see user manual for PWM values

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(int controller_pwm_port) {
    ledController = new Blinkin(controller_pwm_port);
    pwmSetting = -0.99;
    SmartDashboard.putNumber("Blinkin value", pwmSetting);
    SmartDashboard.putData(incrementLEDPatternCommand());
    SmartDashboard.putData(decrementLEDPatternCommand());
    SmartDashboard.putData(bigIncrementLEDPatternCommand());
    SmartDashboard.putData(bigDecrementLEDPatternCommand());
  }

  public CommandBase doNothingCommand() {
    return runOnce( () -> {} );
  }

  /**
   * Factory method for command to increment selected LED pattern.
   *
   * @return a command to increment the LED pattern
   */
  public CommandBase incrementLEDPatternCommand() {
    return runOnce(
        () -> {
          if (pwmSetting < 0.99) {
            pwmSetting += 0.02;
          }
        });
  }

  /**
   * Factory method for command to increment selected LED pattern by 10.
   *
   * @return a command to increment the LED pattern
   */
  public CommandBase bigIncrementLEDPatternCommand() {
    return runOnce(
        () -> {
          if (pwmSetting < 0.79) {
            pwmSetting += 0.2;
          } else {
            pwmSetting = 0.99;
          }
        });
  }

  /**
   * Factory method for command to decrement selected LED pattern.
   *
   * @return a command to decrement the LED pattern
   */
  public CommandBase decrementLEDPatternCommand() {
    return runOnce(
        () -> {
          if (pwmSetting > -0.99) {
            pwmSetting -= 0.02;
          }
        });
  }

  /**
   * Factory method for command to decrement selected LED pattern by 10.
   *
   * @return a command to decrement the LED pattern
   */
  public CommandBase bigDecrementLEDPatternCommand() {
    return runOnce(
        () -> {
          if (pwmSetting > -0.79) {
            pwmSetting -= 0.2;
          } else {
            pwmSetting = -0.99;
          }
        });
  }

  @Override
  public void periodic() {
    ledController.set(pwmSetting);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
