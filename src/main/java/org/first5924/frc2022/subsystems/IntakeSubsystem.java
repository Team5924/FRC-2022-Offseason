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
  private boolean intakeStatus;
  private boolean wheelsStatus;
  private boolean deployed;
  private boolean retracted;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    mIntakeMotor.setNeutralMode(NeutralMode.Brake);
    mState = IntakeState.RETRACTED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Status", intakeStatus);
    SmartDashboard.putBoolean("Wheels Status", wheelsStatus);
    SmartDashboard.putBoolean("Deployed?", deployed);
    SmartDashboard.putBoolean("Retracted", retracted);

    switch (mState) {
      case DEPLOYED:
        deployed = true;
        retracted = false;
        break;
      case RETRACTED:
        deployed = false;
        retracted = true;
        break;
    }
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
    mIntakeMotor.setVoltage(volts);
    intakeStatus = true;
  }

  public void stopIntake() {
    mIntakeMotor.stopMotor();
    intakeStatus = false;
  }

  public void flutterBreak() {
    mIntakeMotor.setVoltage(0.8);
    mIntakeMotor.stopMotor();
    intakeStatus = false;
  }

  // ===== Wheels =====

  public void runIntakeWheels(double speed) {
    mIntakeWheels.set(speed);
    wheelsStatus = true;
  }

  public void stopWheels() {
    mIntakeWheels.stopMotor();
    wheelsStatus = false;
  }
}
