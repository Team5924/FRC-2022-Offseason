// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.first5924.frc2022.constants.DriveConstants;
import org.first5924.frc2022.constants.RobotConstants;
import org.first5924.lib.drivers.TalonFXFactory;
import org.first5924.lib.util.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private AHRS mAhrs;
  private double mGyroOffset = 0;

  private final WPI_TalonFX mLeftFront = TalonFXFactory.createDefaultTalon(DriveConstants.kLeftFrontDrive);
  private final WPI_TalonFX mLeftBack = TalonFXFactory.createDefaultTalon(DriveConstants.kLeftBackDrive);
  private final WPI_TalonFX mRightFront = TalonFXFactory.createDefaultTalon(DriveConstants.kRightFrontDrive);
  private final WPI_TalonFX mRightBack = TalonFXFactory.createDefaultTalon(DriveConstants.kRightBackDrive);

  private final DifferentialDriveOdometry mOdometry;
  private final SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  // private DifferentialDriveWheelSpeeds mPrevSpeeds = new DifferentialDriveWheelSpeeds();
  // private double mPrevTime = -1;

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    try {
      mAhrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    mOdometry = new DifferentialDriveOdometry(getOffsetRotation2d());

    mLeftFront.configNeutralDeadband(0.001);
    mLeftFront.setNeutralMode(NeutralMode.Brake);
    mLeftFront.setInverted(TalonFXInvertType.CounterClockwise);
    mLeftFront.config_kP(0, DriveConstants.kP);

    mRightFront.configNeutralDeadband(0.001);
    mRightFront.setNeutralMode(NeutralMode.Brake);
    mRightFront.setInverted(TalonFXInvertType.Clockwise);
    mRightFront.config_kP(0, DriveConstants.kP);

    mLeftBack.follow(mLeftFront);
    mLeftBack.setInverted(TalonFXInvertType.FollowMaster);

    mRightBack.follow(mRightFront);
    mRightBack.setInverted(TalonFXInvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getOffsetRotation2d().getDegrees());
    SmartDashboard.putNumber("Left Drive Encoder", getLeftPosition());
    SmartDashboard.putNumber("Right Drive Encoder", getRightPosition());

    SmartDashboard.putNumber("Odometry X", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rotation", mOdometry.getPoseMeters().getRotation().getDegrees());

    mOdometry.update(getOffsetRotation2d(), Conversions.sensorUnitsToMeters(getLeftPosition(), DriveConstants.kWheelCircumference) / DriveConstants.kGearboxRatio, Conversions.sensorUnitsToMeters(getRightPosition(), DriveConstants.kWheelCircumference) / DriveConstants.kGearboxRatio);
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Conversions.falconToMPS(getLeftVelocity(), DriveConstants.kWheelCircumference) / DriveConstants.kGearboxRatio,
      Conversions.falconToMPS(getRightVelocity(), DriveConstants.kWheelCircumference) / DriveConstants.kGearboxRatio
    );
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public Rotation2d getOffsetRotation2d() {
    return new Rotation2d(Units.degreesToRadians(-mAhrs.getRotation2d().getDegrees() + mGyroOffset));
  }

  public void setOdometryToPose(Pose2d pose) {
    mGyroOffset = pose.getRotation().getDegrees() - (-mAhrs.getRotation2d().getDegrees());
    mLeftFront.setSelectedSensorPosition(0);
    mRightFront.setSelectedSensorPosition(0);
    mOdometry.resetPosition(pose, getOffsetRotation2d());
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

  // Parts of this method taken from WPILib's DifferentialDrive class
  public void curvatureDrive(double leftJoystickY, double rightJoystickX) {
    double xSpeed = joystickToOutput(-leftJoystickY, 0.08, 0.8);
    double zRotation = joystickToOutput(rightJoystickX, 0.08, 0.65);

    double leftSpeedPercent = xSpeed + (Math.abs(xSpeed) * zRotation);
    double rightSpeedPercent = xSpeed - (Math.abs(xSpeed) * zRotation);

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeedPercent), Math.abs(rightSpeedPercent));
    if (maxMagnitude > 1.0) {
      leftSpeedPercent /= maxMagnitude;
      rightSpeedPercent /= maxMagnitude;
    }

    drivePercent(leftSpeedPercent, rightSpeedPercent);
  }

  // Parts of this method taken from WPILib's DifferentialDrive class
  public void turnInPlace(double leftJoystickY, double rightJoystickX) {
    double xSpeed = -joystickToOutput(leftJoystickY, 0.08, 0.8);
    double zRotation = joystickToOutput(rightJoystickX, 0.08, 0.65);

    double leftSpeedPercent = xSpeed + zRotation;
    double rightSpeedPercent = xSpeed - zRotation;

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeedPercent), Math.abs(rightSpeedPercent));
    if (maxMagnitude > 1.0) {
      leftSpeedPercent /= maxMagnitude;
      rightSpeedPercent /= maxMagnitude;
    }

    drivePercent(leftSpeedPercent, rightSpeedPercent);
  }

  public void drivePercent(double leftPercent, double rightPercent) {
    double leftMotorRPM = leftPercent * RobotConstants.kMaxFalconRPM;
    double rightMotorRPM = rightPercent * RobotConstants.kMaxFalconRPM;

    double leftDriveRotationsPerSecond = Conversions.RPMToRotationsPerSecond(leftMotorRPM) / DriveConstants.kGearboxRatio;
    double rightDriveRotationsPerSecond = Conversions.RPMToRotationsPerSecond(rightMotorRPM) / DriveConstants.kGearboxRatio;

    double leftSpeedFalcon = Conversions.RPMToFalcon(leftMotorRPM);
    double rightSpeedFalcon = Conversions.RPMToFalcon(rightMotorRPM);

    SmartDashboard.putNumber("Left Drive Setpoint", leftSpeedFalcon);
    SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);
    SmartDashboard.putNumber("Right Drive Velocity", getRightVelocity());

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(leftDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(rightDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
  }

  public void driveMPS(double leftMPS, double rightMPS) {
    double leftSpeedFalcon = Conversions.MPSToFalcon(leftMPS, DriveConstants.kWheelCircumference) * DriveConstants.kGearboxRatio;
    double rightSpeedFalcon = Conversions.MPSToFalcon(rightMPS, DriveConstants.kWheelCircumference) * DriveConstants.kGearboxRatio;

    double leftDriveRotationsPerSecond = Conversions.MPSToRotationsPerSecond(leftMPS, DriveConstants.kWheelCircumference);
    double rightDriveRotationsPerSecond = Conversions.MPSToRotationsPerSecond(rightMPS, DriveConstants.kWheelCircumference);

    SmartDashboard.putNumber("Left Drive Setpoint", leftSpeedFalcon);
    SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);
    SmartDashboard.putNumber("Right Drive Velocity", getRightVelocity());

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(leftDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(rightDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
  }

  public void stopDrive() {
    mLeftFront.stopMotor();
    mRightFront.stopMotor();
  }
}
