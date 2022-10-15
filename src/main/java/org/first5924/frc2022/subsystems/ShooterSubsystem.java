// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first5924.frc2022.constants.ShooterConstants;
import org.first5924.frc2022.states.ShooterState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax mLeaderShooterSpark = new CANSparkMax(ShooterConstants.kLeaderShooterSparkPort, MotorType.kBrushless);
  private CANSparkMax mFollowerShooterSpark = new CANSparkMax(ShooterConstants.kFollowerShooterSparkPort, MotorType.kBrushless);

  private RelativeEncoder mShooterEncoder;

  private PIDController mShooterController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private SimpleMotorFeedforward mShooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.ka);

  private ShooterState mState = ShooterState.RUNNING;

  private double mRPMSetpoint;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mFollowerShooterSpark.follow(mLeaderShooterSpark, true);

    mShooterEncoder = mLeaderShooterSpark.getEncoder();
    mShooterEncoder.setVelocityConversionFactor(1/60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ShooterState getState() {
    return mState;
  }

  public void setState(ShooterState state) {
    mState = state;
  }

  public boolean isShooterAtSpeed() {
    return Math.abs(getShooterRPM() - mRPMSetpoint) < 50;
  }

  public double getShooterRPM() {
    return mShooterEncoder.getVelocity();
  }

  public void setShooterSpeed(double RPM) {
    mRPMSetpoint = RPM;
    mLeaderShooterSpark.setVoltage(mShooterController.calculate(getShooterRPM(), RPM) + mShooterFeedforward.calculate(RPM));
  }

  public void setVoltage(double voltage) {
    mLeaderShooterSpark.setVoltage(voltage);
  }

  public void stopShooter() {
    mLeaderShooterSpark.stopMotor();
  }
}
