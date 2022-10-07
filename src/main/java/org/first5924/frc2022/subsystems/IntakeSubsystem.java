// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import org.first5924.frc2022.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX mIntakeTalon = new WPI_TalonFX(IntakeConstants.kIntakeTalon);

  // Troubleshoot variables - TEMP
  private boolean status;
  private boolean forward;
  private boolean backward;
  private double intakeCurrent;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40;
    config.supplyCurrLimit.currentLimit = 30;
    mIntakeTalon.configAllSettings(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeCurrent = mIntakeTalon.getStatorCurrent();

    SmartDashboard.putBoolean("Status", status);
    SmartDashboard.putBoolean("Forward", forward);
    SmartDashboard.putBoolean("Backward", backward);
    SmartDashboard.putNumber("Intake Current", intakeCurrent);
  }

  public void deploy(double volts) {
    mIntakeTalon.setVoltage(volts);
    status = true; forward = true; backward = false;
  }

  public void retract(double volts) {
    mIntakeTalon.setVoltage(volts);
    status = true; forward = false; backward = true;
  }

  public void stop() {
    mIntakeTalon.stopMotor();
    status = false; forward = false; backward = false;
  }
}
