// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.first5924.frc2022.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX mIntakeMotor = new WPI_TalonFX(IntakeConstants.kIntakeTalon);

  // Troubleshoot variables - TEMP
  private boolean runningStatus;
  private boolean deployed;
  private boolean retracted;
  private boolean clearance;

  private double intakeCurrent;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    deployed = false; retracted = true;
    mIntakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeCurrent = mIntakeMotor.getStatorCurrent();
    clearance();

    SmartDashboard.putBoolean("Running?", runningStatus);
    SmartDashboard.putBoolean("Deployed?", deployed);
    SmartDashboard.putBoolean("Retracted?", retracted);
    SmartDashboard.putBoolean("Clearance", clearance);
    System.out.println(intakeCurrent);
  }

  /**
   * "clearance()" returns false when intake is fully deployed/retracted
   */
  public void clearance() {
    // If current spikes to >=5, the intake has reached the physical limit; stop.
    if (intakeCurrent >= 5) {
      clearance = false;
    }
  }

  public void deploy(double volts) {
    if (clearance && !deployed) {
      mIntakeMotor.setVoltage(volts);
      runningStatus = true;
    }
    if (!clearance) {
      deployed = true; retracted = false;
      stop();
      flutterBreak();
    }
  }

  public void retract(double volts) {
    if (clearance && !retracted) {
      mIntakeMotor.setVoltage(-volts);
      runningStatus = true;
    }
    if (!clearance) {
      deployed = false; retracted = true;
      mIntakeMotor.setNeutralMode(NeutralMode.Brake);
      stop();
    }
  }

  public void stop() {
    clearance = true; runningStatus = false;
    mIntakeMotor.stopMotor();
  }

  public void flutterBreak() {
    clearance = true; runningStatus = false;
    mIntakeMotor.setNeutralMode(NeutralMode.Coast);
    mIntakeMotor.setVoltage(0.8);
    mIntakeMotor.stopMotor();
  }
}
