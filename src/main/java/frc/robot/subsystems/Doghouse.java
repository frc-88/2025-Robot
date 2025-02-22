// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Doghouse extends SubsystemBase {
  /** Creates a new Doghouse. */
  private DoublePreferenceConstant p_funnelSlowSpeed =
      new DoublePreferenceConstant("Doghouse/FunnelSlowSpeed", 0.2);

  private DoublePreferenceConstant p_funnelFastSpeed =
      new DoublePreferenceConstant("Doghouse/Funnel/FastSpeed", 1);
  private DoublePreferenceConstant p_funnelCurrentLimit =
      new DoublePreferenceConstant("Doghouse/Funnel/CurrentLimit", 20);
  private DoublePreferenceConstant p_manipulatorInSpeed =
      new DoublePreferenceConstant("Doghouse/Manipulator/InSpeed", 0.3);
  private DoublePreferenceConstant p_manipulatorOutSpeed =
      new DoublePreferenceConstant("Doghouse/Manipulator/OutSpeed", 0.3);
  private DoublePreferenceConstant p_manipulatorCurrentLimit =
      new DoublePreferenceConstant("Doghouse/Manipulator/CurrentLimit", 20);

  private final DutyCycleOut m_funnelRequest = new DutyCycleOut(0.0);

  private TalonFX m_funnel = new TalonFX(Constants.DOGHOUSE_FUNNEL_MOTOR, "rio");
  private TalonFX m_manipulator =
      new TalonFX(Constants.ELEVATOR_MANIPULATOR_MOTOR, Constants.RIO_CANBUS);

  private final CANrange m_doghousCANRange =
      new CANrange(Constants.DOGHOUSE_CANRANGE, Constants.RIO_CANBUS);
  private final CANrange m_coralRange =
      new CANrange(Constants.CORAL_CANRANGE, Constants.RIO_CANBUS);

  private boolean hasCoral = false;
  private boolean isIn = false;

  public Trigger hasCoralTrigger = new Trigger(() -> hasCoral);
  public Trigger isInTrigger = new Trigger(() -> isIn);

  public Doghouse() {
    // configure funnel
    TalonFXConfiguration doghouseConfiguration = new TalonFXConfiguration();
    doghouseConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimit = p_funnelCurrentLimit.getValue();
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_funnel.getConfigurator().apply(doghouseConfiguration);

    // configure manipulator
    TalonFXConfiguration manipulatorConfiguration = new TalonFXConfiguration();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimit =
        p_manipulatorCurrentLimit.getValue();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_manipulator.getConfigurator().apply(manipulatorConfiguration);

    configureCANrange();
  }

  private void configureCANrange() {
    CANrangeConfiguration coralRangecfg = new CANrangeConfiguration();
    CANrangeConfiguration doghouscfg = new CANrangeConfiguration();
    // CANrangeConfiguration canRangerightcfg = new CANrangeConfiguration();
    coralRangecfg.FovParams.FOVRangeX = 6.75;
    doghouscfg.FovParams.FOVRangeX = 7.5;
    coralRangecfg.FovParams.FOVRangeY = 6.75;
    doghouscfg.FovParams.FOVRangeY = 27.0;
    // canRangerightcfg.FovParams.FOVRangeY = 27.0;

    doghouscfg.ToFParams.UpdateFrequency = 50;
    coralRangecfg.ToFParams.UpdateFrequency = 50;

    doghouscfg.ProximityParams.ProximityThreshold = 0.5;
    doghouscfg.ProximityParams.ProximityHysteresis = 0.03;
    coralRangecfg.ProximityParams.ProximityThreshold = 0.5;

    coralRangecfg.ProximityParams.MinSignalStrengthForValidMeasurement = 40000;
    doghouscfg.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;

    m_doghousCANRange.getConfigurator().apply(doghouscfg);
    // m_canRangeRight.getConfigurator().apply(canRangerightcfg);
    m_coralRange.getConfigurator().apply(coralRangecfg);
  }

  public boolean isCoralDetected() {
    return m_coralRange.getIsDetected().getValue();
  }

  public boolean isDoghouseDetected() {
    return m_doghousCANRange.getIsDetected().getValue();
  }

  public Trigger hasCoral() {
    return hasCoralTrigger;
  }

  public Trigger isIn() {
    return isInTrigger;
  }

  private void setSpeed(double output) {
    m_funnel.setControl(m_funnelRequest.withOutput(output));
  }

  private void stopMoving() {
    setSpeed(0.0);
  }

  private void moveSlow() {
    setSpeed(p_funnelSlowSpeed.getValue());
  }

  private void moveFast() {
    setSpeed(p_funnelFastSpeed.getValue());
  }

  private void manipulatorStop() {
    m_manipulator.setControl(new DutyCycleOut(0.0));
  }

  private void manipulatorIn() {
    m_manipulator.setControl(new DutyCycleOut(p_manipulatorInSpeed.getValue()));
  }

  private void manipulatorOut() {
    m_manipulator.setControl(new DutyCycleOut(p_manipulatorOutSpeed.getValue()));
  }

  private void manipulatorReverse() {
    m_manipulator.setControl(new DutyCycleOut(0.1));
  }

  private void manipulatorSlow() {
    m_manipulator.setControl(new DutyCycleOut(-0.1));
  }

  private void algaePickup() {
    m_manipulator.setControl(new DutyCycleOut(1.0));
  }

  public Command stopMovingFactory() {
    return new RunCommand(() -> stopMoving(), this);
  }

  public Command moveSlowFactory() {
    return new RunCommand(() -> moveSlow(), this);
  }

  public Command algaePickupFactory() {
    return new RunCommand(() -> algaePickup(), this);
  }

  public Command coralIntake() {
    return new RunCommand(
        () -> {
          moveFast();
          if (!m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorIn();
          } else if (m_coralRange.getIsDetected().getValue()
              && m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorSlow();
          } else if (m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorStop();
          }
        },
        this);
  }

  public Command manipulatorOutFactory() {
    return new RunCommand(() -> manipulatorOut(), this)
        .withTimeout(1.0)
        .andThen(
            () -> {
              manipulatorStop();
              hasCoral = false;
              isIn = false;
            });
  }

  public Command backUpFactory() {
    return new RunCommand(() -> manipulatorReverse(), this)
        .until(() -> !m_coralRange.getIsDetected().getValue())
        .andThen(
            () -> {
              manipulatorStop();
              isIn = true;
            });
  }

  public Command manipulatorInFactory() {
    return new RunCommand(
        () -> {
          if (!m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorIn();
          } else if (m_coralRange.getIsDetected().getValue()
              && m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorSlow();
          } else if (m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorStop();
          }
        },
        this);
  }

  public Command manipulatorStopFactory() {
    return new InstantCommand(() -> manipulatorStop(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Doghouse/Funnnel/Current", m_funnel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean(
        "Doghouse/CoralRange/Is detected", m_coralRange.getIsDetected().getValue());
    SmartDashboard.putNumber(
        "Doghouse/FenceRange/Distance",
        Units.metersToInches(m_doghousCANRange.getDistance().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Doghouse/CoralRange/Distance",
        Units.metersToInches(m_coralRange.getDistance().getValueAsDouble()));
    SmartDashboard.putBoolean(
        "Doghouse/FenceRange/Is detected", m_doghousCANRange.getIsDetected().getValue());
  }
}
