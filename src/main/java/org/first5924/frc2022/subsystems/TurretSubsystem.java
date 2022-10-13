// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.first5924.frc2022.constants.RobotConstants;
import org.first5924.frc2022.constants.TurretConstants;
import org.first5924.frc2022.states.TurretState;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax mTurretSpark = new CANSparkMax(TurretConstants.kSparkPort, MotorType.kBrushless);
  private final PIDController mTurretController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
  private final RelativeEncoder mTurretEncoder;

  private TurretState mState = TurretState.TRACKING;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    mTurretSpark.restoreFactoryDefaults();
    mTurretSpark.enableVoltageCompensation(RobotConstants.kNominalVoltage);
    mTurretSpark.enableSoftLimit(SoftLimitDirection.kForward, true);
    mTurretSpark.enableSoftLimit(SoftLimitDirection.kReverse, true);
    mTurretSpark.setSoftLimit(SoftLimitDirection.kForward, (float) (TurretConstants.kRangeOfMotion / 2 * TurretConstants.kGearRatio));
    mTurretSpark.setSoftLimit(SoftLimitDirection.kReverse, (float) (-TurretConstants.kRangeOfMotion / 2 * TurretConstants.kGearRatio));
    mTurretController.setTolerance(TurretConstants.kPositionTolerance, TurretConstants.kVelocityTolerance);
    mTurretEncoder = mTurretSpark.getEncoder();
    mTurretEncoder.setPositionConversionFactor(360);
    mTurretEncoder.setVelocityConversionFactor(1/60);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Velo", getTurretVelocity());
    SmartDashboard.putNumber("Turret Pos", getTurretPosition());
    SmartDashboard.putNumber("Neo Pos", getPosition());
    SmartDashboard.putString("State", getState().toString());
  }

  public TurretState getState() {
    return mState;
  }

  public void setState(TurretState state) {
    mState = state;
  }

  public void zeroTurret() {
    mTurretEncoder.setPosition(0);
  }

  public double getTurretVelocity() {
    return getVelocity() / TurretConstants.kGearRatio;
  }

  public double getTurretPosition() {
    return getPosition() / TurretConstants.kGearRatio;
  }

  public double getVelocity() {
    return mTurretEncoder.getVelocity();
  }

  public double getPosition() {
    return mTurretEncoder.getPosition();
  }

  public void turnTurretToDegrees(double turretSetpointDegrees) {
    double neoSetpointDegrees = turretSetpointDegrees * TurretConstants.kGearRatio;
    double voltage = MathUtil.clamp(mTurretController.calculate(mTurretEncoder.getPosition(), neoSetpointDegrees), -1, 1);
    mTurretSpark.setVoltage(voltage);
    SmartDashboard.putNumber("Turret Voltage", voltage);
    SmartDashboard.putNumber("Neo Setpoint", neoSetpointDegrees);
  }

  public boolean isTurretAtSetpoint() {
    return mTurretController.atSetpoint();
  }

  public void setVoltage(double voltage) {
    mTurretSpark.setVoltage(voltage);
  }
}
