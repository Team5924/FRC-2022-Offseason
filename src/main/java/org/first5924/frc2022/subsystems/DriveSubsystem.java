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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private AHRS ahrs;

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
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    mOdometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

    mLeftFront.configNeutralDeadband(0.001);
    mLeftFront.setNeutralMode(NeutralMode.Brake);
    mLeftFront.setInverted(TalonFXInvertType.Clockwise);

    mRightFront.configNeutralDeadband(0.001);
    mRightFront.setNeutralMode(NeutralMode.Brake);
    mRightFront.setInverted(TalonFXInvertType.CounterClockwise);

    mLeftBack.follow(mLeftFront);
    mLeftBack.setInverted(TalonFXInvertType.FollowMaster);

    mRightBack.follow(mRightFront);
    mRightBack.setInverted(TalonFXInvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    mOdometry.update(ahrs.getRotation2d(), Conversions.sensorUnitsToRobotMeters(getLeftPosition(), DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches), Conversions.sensorUnitsToRobotMeters(getRightPosition(), DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches));
  }

  public double getLeftVelocity() {
    return mLeftFront.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return mRightFront.getSelectedSensorVelocity();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Conversions.falconToRobotMPS(getLeftVelocity(), DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches), Conversions.falconToRobotMPS(getRightVelocity(), DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches));
  }

  public double getLeftPosition() {
    return mLeftFront.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return mRightFront.getSelectedSensorPosition();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
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

    SmartDashboard.putNumber("Left Joystick", leftJoystickY);
    SmartDashboard.putNumber("Right Joystick", rightJoystickX);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("zRotation", zRotation);

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
    double leftRPM = leftPercent * DriveConstants.kMaxRPM;
    double rightRPM = rightPercent * DriveConstants.kMaxRPM;

    double leftRotationsPerSecond = Conversions.RPMToRotationsPerSecond(leftRPM);
    double rightRotationsPerSecond = Conversions.RPMToRotationsPerSecond(rightRPM);

    double leftSpeedFalcon = Conversions.RPMToFalcon(leftRPM);
    double rightSpeedFalcon = Conversions.RPMToFalcon(rightRPM);

    SmartDashboard.putNumber("Left Drive Setpoint", leftSpeedFalcon);
    SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);
    SmartDashboard.putNumber("Right Drive Velocity", getRightVelocity());

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(leftRotationsPerSecond) / RobotConstants.kNominalVoltage);
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(rightRotationsPerSecond) / RobotConstants.kNominalVoltage);
  }

  public void driveMPS(double leftMPS, double rightMPS) {
    double leftSpeedFalcon = Conversions.robotMPSToFalcon(leftMPS, DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches);
    double rightSpeedFalcon = Conversions.robotMPSToFalcon(rightMPS, DriveConstants.kGearboxRatio, DriveConstants.kWheelCircumferenceInches);

    double leftRotationsPerSecond = Conversions.falconToRotationsPerSecond(leftSpeedFalcon);
    double rightRotationsPerSecond = Conversions.falconToRotationsPerSecond(rightSpeedFalcon);

    SmartDashboard.putNumber("Left Drive Setpoint", leftSpeedFalcon);
    SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);
    SmartDashboard.putNumber("Right Drive Velocity", getRightVelocity());

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(leftRotationsPerSecond) / RobotConstants.kNominalVoltage);
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(rightRotationsPerSecond) / RobotConstants.kNominalVoltage);
  }
}
