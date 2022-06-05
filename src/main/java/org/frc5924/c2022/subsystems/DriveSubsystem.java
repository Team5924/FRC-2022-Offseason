// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5924.c2022.Constants.DriveConstants;
import org.frc5924.c2022.Constants.OIConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX mLeftFront = new WPI_TalonFX(DriveConstants.kLeftFront);
  private final WPI_TalonFX mLeftBack = new WPI_TalonFX(DriveConstants.kLeftBack);
  private final WPI_TalonFX mRightFront = new WPI_TalonFX(DriveConstants.kRightFront);
  private final WPI_TalonFX mRightBack = new WPI_TalonFX(DriveConstants.kRightBack);

  private final SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.neutralDeadband = 0.001; // Smallest allowed neutral deadband
    driveConfig.nominalOutputForward = 0;
    driveConfig.nominalOutputReverse = 0;
    driveConfig.peakOutputForward = 1;
    driveConfig.peakOutputReverse = -1;
    driveConfig.voltageCompSaturation = 11;
    driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 43, 57.5, 1.5);

    mLeftFront.configAllSettings(driveConfig);
    mLeftFront.setNeutralMode(NeutralMode.Brake);
    mLeftFront.enableVoltageCompensation(true);
    mLeftFront.setInverted(TalonFXInvertType.Clockwise);
    mLeftFront.set(ControlMode.Velocity, 0);

    mRightFront.configAllSettings(driveConfig);
    mRightFront.setNeutralMode(NeutralMode.Brake);
    mRightFront.enableVoltageCompensation(true);
    mRightFront.setInverted(TalonFXInvertType.CounterClockwise);
    mRightFront.set(ControlMode.Velocity, 0);

    mLeftBack.follow(mLeftFront);
    mLeftBack.setInverted(TalonFXInvertType.FollowMaster);

    mRightBack.follow(mRightFront);
    mRightBack.setInverted(TalonFXInvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
  }

  public double getLeftVelocity() {
    return mLeftFront.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return mRightFront.getSelectedSensorVelocity();
  }

  public double getLeftPosition() {
    return mLeftFront.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return mRightFront.getSelectedSensorPosition();
  }

  public void stopLeft() {
    mLeftFront.stopMotor();
  }

  public void stopRight() {
    mRightFront.stopMotor();
  }

  public void curvatureDrive(double leftJoystickY, double rightJoystickX, boolean allowTurnInPlace) {
    double xSpeed = -joystickToOutput(leftJoystickY, 0.03, 0.8);
    double zRotation = joystickToOutput(rightJoystickX, 0.03, 0.65);
    WheelSpeeds speeds = curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace);
    double leftSetpoint = speeds.left * DriveConstants.kMaxSpeed;
    double rightSetpoint = speeds.right * DriveConstants.kMaxSpeed;
    mLeftFront.set(ControlMode.Velocity, leftSetpoint, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(leftSetpoint));
    mRightFront.set(ControlMode.Velocity, rightSetpoint, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(rightSetpoint));
  }

  // Taken from WPILib's DifferentialDrive class
  private WheelSpeeds curvatureDriveIK(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double leftSpeed;
    double rightSpeed;

    if (allowTurnInPlace) {
      leftSpeed = xSpeed + zRotation;
      rightSpeed = xSpeed - zRotation;
    } else {
      leftSpeed = xSpeed + Math.abs(xSpeed) * zRotation;
      rightSpeed = xSpeed - Math.abs(xSpeed) * zRotation;
    }

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }

    return new WheelSpeeds(leftSpeed, rightSpeed);
  }

  // https://www.desmos.com/calculator/4dkyeczdx6
  private double joystickToOutput(double joystick, double deadzone, double maxOutput) {
    if (joystick < -deadzone) {
      return maxOutput / (1 - deadzone) * (joystick + deadzone);
    } else if (joystick > deadzone) {
      return maxOutput / (1 - deadzone) * (joystick - deadzone);
    } else {
      return 0;
    }
  }
}
