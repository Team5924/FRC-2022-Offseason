// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.first5924.frc2022.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX mIntakeMotor = new WPI_TalonFX(IntakeConstants.kIntakeTalon);

  private boolean runningStatus;
  private Status status;
  private boolean clearance;

  enum Status {
    DEPLOYED,
    RETRACTED
  }

  private double intakeCurrent;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    status = Status.RETRACTED;
    mIntakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeCurrent = mIntakeMotor.getStatorCurrent();
    clearance();

    SmartDashboard.putBoolean("Running?", runningStatus);
    SmartDashboard.putBoolean("Clearance", clearance);

    System.out.println(intakeCurrent);
  }

  // ===== Intake =====

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
    if (clearance && status == Status.RETRACTED) {
      mIntakeMotor.setVoltage(volts);
      runningStatus = true;
    }
    if (!clearance) {
      status = Status.DEPLOYED;
      stopIntake();
      flutterBreak();
    }
  }

  public void retract(double volts) {
    if (clearance && status == Status.DEPLOYED) {
      mIntakeMotor.setVoltage(-volts);
      runningStatus = true;
    }
    if (!clearance) {
      status = Status.RETRACTED;
      mIntakeMotor.setNeutralMode(NeutralMode.Brake);
      stopIntake();
    }
  }

  public void stopIntake() {
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
