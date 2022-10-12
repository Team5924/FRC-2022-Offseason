// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.first5924.frc2022.constants.IntakeConstants;

import org.first5924.frc2022.states.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
  // Instances
  private WPI_TalonFX mIntakeMotor = new WPI_TalonFX(IntakeConstants.kIntakeTalon);
  private CANSparkMax mIntakeWheels = new CANSparkMax(IntakeConstants.kIntakeWheels, MotorType.kBrushless);

  // Variables
  private IntakeState mState;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    mIntakeMotor.setNeutralMode(NeutralMode.Brake);
    mState = IntakeState.RETRACTED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake State?", getState().toString());
    SmartDashboard.putNumber("Current", getIntakeCurrent());
  }

  public void setNeutralMode(NeutralMode mode) {
    mIntakeMotor.setNeutralMode(mode);
  }

  public IntakeState getState() {
    return mState;
  }

  public void setState(IntakeState state) {
    mState = state;
  }

  // ===== Intake =====

  public void runIntake(double volts) {
    mIntakeMotor.setVoltage(volts);
  }

  public void stopIntake() {
    mIntakeMotor.stopMotor();
  }

  public double getIntakeCurrent() {
    return mIntakeMotor.getStatorCurrent();
  }

  // ===== Wheels =====

  public void runWheels(double speed) {
    mIntakeWheels.set(-speed);
  }

  public void stopWheels() {
    mIntakeWheels.stopMotor();
  }
}
