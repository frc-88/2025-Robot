// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Doghouse extends SubsystemBase {
  /** Creates a new Doghouse. */
  private DoublePreferenceConstant p_funnelSlowSpeed =
      new DoublePreferenceConstant("Doghouse/FunnelSlowSpeed", 0.2);

  private DoublePreferenceConstant p_funnelFastSpeed =
      new DoublePreferenceConstant("Doghouse/FunnelFastSpeed", 1);
  private DoublePreferenceConstant p_funnelCurrentLimit =
      new DoublePreferenceConstant("Doghouse/FunnelCurrentLimit", 20);

  private final DutyCycleOut m_funnelRequest = new DutyCycleOut(0.0);

  private TalonFX m_funnel = new TalonFX(Constants.DOGHOUSE_FUNNEL_MOTOR, "rio");

  public Doghouse() {
    TalonFXConfiguration doghouseConfiguration = new TalonFXConfiguration();
    doghouseConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimit = p_funnelCurrentLimit.getValue();
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    doghouseConfiguration.OpenLoopRamps =
        new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0);
    m_funnel.getConfigurator().apply(doghouseConfiguration);
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

  public Command stopMovingFactory() {
    return new RunCommand(() -> stopMoving(), this);
  }

  public Command moveSlowFactory() {
    return new RunCommand(() -> moveSlow(), this);
  }

  public Command moveFastFactory() {
    return new RunCommand(() -> moveFast(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Doghouse/Current", m_funnel.getStatorCurrent().getValueAsDouble());
  }
}
