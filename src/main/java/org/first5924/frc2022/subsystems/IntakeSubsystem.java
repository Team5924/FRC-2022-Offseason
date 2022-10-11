// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.first5924.frc2022.constants.IntakeConstants;

import org.first5924.frc2022.states.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX mIntakeMotor = new WPI_TalonFX(IntakeConstants.kIntakeTalon);
  private IntakeState mState;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    mIntakeMotor.setNeutralMode(NeutralMode.Brake);
    mState = IntakeState.RETRACTED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ===== Intake =====

  public double getCurrent() {
    return mIntakeMotor.getStatorCurrent();
  }

  public IntakeState getState() {
    return mState;
  }

  public void setState(IntakeState state) {
    mState = state;
  }

  public void runIntakeMotor(double volts) {
    mIntakeMotor.set(volts);
  }

  public void stopIntake() {
    mIntakeMotor.stopMotor();
  }

  public void flutterBreak() {
    mIntakeMotor.setVoltage(0.8);
    mIntakeMotor.stopMotor();
  }
}
