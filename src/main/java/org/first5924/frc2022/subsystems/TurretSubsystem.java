// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first5924.frc2022.constants.RobotConstants;
import org.first5924.frc2022.constants.TurretConstants;
import org.first5924.frc2022.states.TurretState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax mTurretSpark = new CANSparkMax(TurretConstants.kSparkPort, MotorType.kBrushless);
  private final ProfiledPIDController mTurretController = new ProfiledPIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, new TrapezoidProfile.Constraints(TurretConstants.kMaxVel, TurretConstants.kMaxAccel));
  private final SimpleMotorFeedforward mTurretFeedforward = new SimpleMotorFeedforward(TurretConstants.ks, TurretConstants.kv, TurretConstants.ka);
  private final RelativeEncoder mTurretEncoder;

  private TurretState mState = TurretState.TRACKING;

  private double mLastSpeed = 0;
  private double mLastTime = Timer.getFPGATimestamp();

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    mTurretSpark.restoreFactoryDefaults();
    mTurretSpark.enableVoltageCompensation(RobotConstants.kNominalVoltage);
    mTurretSpark.setSoftLimit(SoftLimitDirection.kForward, (float) (TurretConstants.kRangeOfMotion / 360 / 2));
    mTurretSpark.setSoftLimit(SoftLimitDirection.kReverse, (float) (-TurretConstants.kRangeOfMotion / 360 / 2));
    mTurretController.setTolerance(TurretConstants.kAllowedError);
    mTurretEncoder = mTurretSpark.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Velo", getVelocity());
    SmartDashboard.putNumber("Turret Pos", getPosition());
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
    return mTurretEncoder.getPosition() / TurretConstants.kGearRatio;
  }

  public double getVelocity() {
    return mTurretEncoder.getVelocity() / 60;
  }

  public double getPosition() {
    return mTurretEncoder.getPosition();
  }

  public void turnToDegrees(double goalDegrees) {
    double acceleration = (mTurretController.getSetpoint().velocity - mLastSpeed) / (Timer.getFPGATimestamp() - mLastTime);
    double neoRotationError = goalDegrees / 360 * TurretConstants.kGearRatio;
    mTurretSpark.setVoltage(mTurretController.calculate(getPosition(), neoRotationError) + mTurretFeedforward.calculate(mTurretController.getSetpoint().velocity, acceleration));
    mLastSpeed = mTurretController.getSetpoint().velocity;
    mLastTime = Timer.getFPGATimestamp();
  }

  public void fuckingFreeze() {
    mTurretSpark.stopMotor();
  }
}
