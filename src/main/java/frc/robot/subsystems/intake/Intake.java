// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final TalonFX motor;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Preferences keys
  private static final String FORWARD_DUTY_CYCLE_KEY = "Intake/ForwardDutyCycle";
  private static final String BACKWARD_DUTY_CYCLE_KEY = "Intake/BackwardDutyCycle";

  // Status tracking
  private enum Status {
    FORWARD,
    BACKWARDS,
    STOPPED
  }

  private Status currentStatus = Status.STOPPED;

  public Intake(int motorCANId) {
    motor = new TalonFX(motorCANId);

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    // Initialize preferences with default values (10% = 0.1)
    Preferences.initDouble(FORWARD_DUTY_CYCLE_KEY, 0.1);
    Preferences.initDouble(BACKWARD_DUTY_CYCLE_KEY, 0.1);

    // Initialize SmartDashboard values with preference defaults
    SmartDashboard.putNumber("Intake/ForwardDutyCycle", 0.1);
    SmartDashboard.putNumber("Intake/BackwardDutyCycle", 0.1);
  }

  /**
   * Drives the intake forward at the duty cycle set in preferences.
   */
  public void forward() {
    double dutyCycle = Preferences.getDouble(FORWARD_DUTY_CYCLE_KEY, 0.1);
    // Duty cycle is a percentage, so multiply by 12V to get voltage
    double voltage = dutyCycle * 12.0;
    motor.setControl(voltageRequest.withOutput(voltage));
    currentStatus = Status.FORWARD;
  }

  /**
   * Drives the intake backwards at the duty cycle set in preferences.
   */
  public void backward() {
    double dutyCycle = Preferences.getDouble(BACKWARD_DUTY_CYCLE_KEY, 0.1);
    // Duty cycle is a percentage, so multiply by 12V to get voltage (negative for reverse)
    double voltage = -dutyCycle * 12.0;
    motor.setControl(voltageRequest.withOutput(voltage));
    currentStatus = Status.BACKWARDS;
  }

  /**
   * Stops the intake motor.
   */
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
    currentStatus = Status.STOPPED;
  }

  @Override
  public void periodic() {
    // Read current preference values
    double currentForwardDutyCycle = Preferences.getDouble(FORWARD_DUTY_CYCLE_KEY, 0.1);
    double currentBackwardDutyCycle = Preferences.getDouble(BACKWARD_DUTY_CYCLE_KEY, 0.1);

    // Read from SmartDashboard (users can edit these values)
    double dashboardForward = SmartDashboard.getNumber("Intake/ForwardDutyCycle", currentForwardDutyCycle);
    double dashboardBackward = SmartDashboard.getNumber("Intake/BackwardDutyCycle", currentBackwardDutyCycle);

    // Update preferences if SmartDashboard values changed
    if (Math.abs(dashboardForward - currentForwardDutyCycle) > 0.001) {
      Preferences.setDouble(FORWARD_DUTY_CYCLE_KEY, dashboardForward);
    }
    if (Math.abs(dashboardBackward - currentBackwardDutyCycle) > 0.001) {
      Preferences.setDouble(BACKWARD_DUTY_CYCLE_KEY, dashboardBackward);
    }

    // Update SmartDashboard to show current preference values (in case they were loaded from flash)
    SmartDashboard.putNumber("Intake/ForwardDutyCycle", Preferences.getDouble(FORWARD_DUTY_CYCLE_KEY, 0.1));
    SmartDashboard.putNumber("Intake/BackwardDutyCycle", Preferences.getDouble(BACKWARD_DUTY_CYCLE_KEY, 0.1));

    // Report status
    String statusString;
    switch (currentStatus) {
      case FORWARD:
        statusString = "forward";
        break;
      case BACKWARDS:
        statusString = "backwards";
        break;
      case STOPPED:
      default:
        statusString = "stopped";
        break;
    }
    SmartDashboard.putString("Intake/Status", statusString);

    // Report power draw (rounded to 2 decimal places)
    double currentAmps = motor.getStatorCurrent().getValueAsDouble();
    double voltage = motor.getMotorVoltage().getValueAsDouble();
    double powerWatts = currentAmps * voltage;
    SmartDashboard.putNumber("Intake/PowerDraw", Math.round(powerWatts * 100.0) / 100.0);

    // Log to AdvantageKit
    Logger.recordOutput("Intake/Status", statusString);
    Logger.recordOutput("Intake/Power", powerWatts);
    
    // Calculate motor duty cycle from applied voltage (duty cycle = voltage / 12V)
    double motorDutyCycle = voltage / 12.0;
    Logger.recordOutput("Intake/MotorDutyCycle", motorDutyCycle);
  }
}
