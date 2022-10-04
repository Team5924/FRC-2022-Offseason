// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import org.first5924.frc2022.constants.DriveConstants;
import org.first5924.frc2022.constants.RobotConstants;
import org.first5924.lib.drivers.TalonFXFactory;
import org.first5924.lib.util.Conversions;
import org.first5924.lib.util.JoystickToOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private AHRS mAhrs;
  private double mGyroOffset = 0;

  private final Timer mTimer = new Timer();

  private final WPI_TalonFX mLeftFront = TalonFXFactory.createDefaultTalon(DriveConstants.kLeftFrontDrive);
  private final WPI_TalonFX mLeftBack = TalonFXFactory.createDefaultTalon(DriveConstants.kLeftBackDrive);
  private final WPI_TalonFX mRightFront = TalonFXFactory.createDefaultTalon(DriveConstants.kRightFrontDrive);
  private final WPI_TalonFX mRightBack = TalonFXFactory.createDefaultTalon(DriveConstants.kRightBackDrive);

  private final CANCoder mLeftCANCoder = new CANCoder(DriveConstants.kLeftCANCoder);
  private final CANCoder mRightCANCoder = new CANCoder(DriveConstants.kRightCANCoder);

  private final DifferentialDriveOdometry mOdometry;
  private final SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  private final TrapezoidProfile.Constraints mRotationTrapezoidProfileConstraints = new TrapezoidProfile.Constraints(DriveConstants.kRotateMaxSpeed, DriveConstants.kRotateMaxAccel);

  private double mPrevLeftDriveRotationsPerSecond = 0;
  private double mPrevRightDriveRotationsPerSecond = 0;

  private double mPrevTime = 0;

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    mTimer.start();

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

    mLeftCANCoder.configSensorDirection(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getOffsetRotation2d().getDegrees());
    SmartDashboard.putNumber("Left CANCoder", mLeftCANCoder.getPosition());
    SmartDashboard.putNumber("Right CANCoder", mRightCANCoder.getPosition());
    SmartDashboard.putNumber("Left CANCoder Velo", getLeftWheelVelocity());
    SmartDashboard.putNumber("Right CANCoder Velo", getRightWheelVelocity());
    SmartDashboard.putNumber("Left Falcon Velo", getLeftFalconVelocity());
    SmartDashboard.putNumber("Right Falcon Velo", getRightFalconVelocity());

    SmartDashboard.putNumber("Odometry X", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rotation", mOdometry.getPoseMeters().getRotation().getDegrees());

    mOdometry.update(getOffsetRotation2d(), Conversions.degreesToMeters(getLeftWheelPosition(), DriveConstants.kWheelCircumference), Conversions.degreesToMeters(getRightWheelPosition(), DriveConstants.kWheelCircumference));
  }

  public double getLeftFalconVelocity() {
    return mLeftFront.getSelectedSensorVelocity();
  }

  public double getRightFalconVelocity() {
    return mRightFront.getSelectedSensorVelocity();
  }

  public double getLeftFalconPosition() {
    return mLeftFront.getSelectedSensorPosition();
  }

  public double getRightFalconPosition() {
    return mRightFront.getSelectedSensorPosition();
  }

  public double getLeftWheelVelocity() {
    return mLeftCANCoder.getVelocity();
  }

  public double getRightWheelVelocity() {
    return mRightCANCoder.getVelocity();
  }

  public double getLeftWheelPosition() {
    return mRightCANCoder.getPosition();
  }

  public double getRightWheelPosition() {
    return mRightCANCoder.getPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Conversions.degreesPerSecondToMPS(getLeftWheelVelocity(), DriveConstants.kWheelCircumference),
      Conversions.degreesPerSecondToMPS(getRightWheelVelocity(), DriveConstants.kWheelCircumference)
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
    mLeftCANCoder.setPosition(0);
    mRightCANCoder.setPosition(0);
    mOdometry.resetPosition(pose, getOffsetRotation2d());
  }

  // Parts of this method taken from WPILib's DifferentialDrive class
  public void curvatureDrive(double leftJoystickY, double rightJoystickX) {
    double xSpeed = JoystickToOutput.calculateLinear(-leftJoystickY, 0.08, 0.8);
    double zRotation = JoystickToOutput.calculateLinear(rightJoystickX, 0.08, 0.65);

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
    double xSpeed = -JoystickToOutput.calculateLinear(leftJoystickY, 0.08, 0.8);
    double zRotation = JoystickToOutput.calculateLinear(rightJoystickX, 0.08, 0.65);

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
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(leftDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, MathUtil.clamp(mDriveFeedforward.calculate(rightDriveRotationsPerSecond) / RobotConstants.kNominalVoltage, -0.8, 0.8));
  }

  public void driveMPS(double leftMPS, double rightMPS) {
    double curTime = mTimer.get();
    double dt = curTime - mPrevTime;

    double leftSpeedFalcon = Conversions.MPSToFalcon(leftMPS, DriveConstants.kWheelCircumference) * DriveConstants.kGearboxRatio;
    double rightSpeedFalcon = Conversions.MPSToFalcon(rightMPS, DriveConstants.kWheelCircumference) * DriveConstants.kGearboxRatio;

    double leftDriveRotationsPerSecond = Conversions.MPSToRotationsPerSecond(leftMPS, DriveConstants.kWheelCircumference);
    double rightDriveRotationsPerSecond = Conversions.MPSToRotationsPerSecond(rightMPS, DriveConstants.kWheelCircumference);

    SmartDashboard.putNumber("Left Drive Setpoint", leftSpeedFalcon);
    SmartDashboard.putNumber("Right Drive Setpoint", rightSpeedFalcon);

    mLeftFront.set(ControlMode.Velocity, leftSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(leftDriveRotationsPerSecond, (leftDriveRotationsPerSecond - mPrevLeftDriveRotationsPerSecond) / dt) / RobotConstants.kNominalVoltage);
    mRightFront.set(ControlMode.Velocity, rightSpeedFalcon, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(rightDriveRotationsPerSecond, (rightDriveRotationsPerSecond - mPrevRightDriveRotationsPerSecond) / dt) / RobotConstants.kNominalVoltage);

    mPrevTime = curTime;
    mPrevLeftDriveRotationsPerSecond = leftDriveRotationsPerSecond;
    mPrevRightDriveRotationsPerSecond = rightDriveRotationsPerSecond;
  }

  public void gyroRotate(double degrees) {
    double curTime = mTimer.get();
    double dt = curTime - mPrevTime;

    double goalRotationError = degrees - getOffsetRotation2d().getDegrees();
    double goalMetersError = (goalRotationError / 360) * 2 * Math.PI * (DriveConstants.kTrackwidth / 2);
    double goalFalconError = Conversions.metersToFalconUnits(goalMetersError, DriveConstants.kWheelCircumference) * DriveConstants.kGearboxRatio;
    TrapezoidProfile leftRotationTrapezoidProfile = new TrapezoidProfile(mRotationTrapezoidProfileConstraints, new TrapezoidProfile.State(getLeftFalconPosition() - goalFalconError, 0), new TrapezoidProfile.State(getLeftFalconPosition(), getLeftFalconVelocity()));
    TrapezoidProfile rightRotationTrapezoidProfile = new TrapezoidProfile(mRotationTrapezoidProfileConstraints, new TrapezoidProfile.State(getRightFalconPosition() + goalFalconError, 0), new TrapezoidProfile.State(getRightFalconPosition(), getRightFalconVelocity()));
    // 0.02 is the how frequently the command execute method runs
    TrapezoidProfile.State leftSetpoint = leftRotationTrapezoidProfile.calculate(0.02);
    TrapezoidProfile.State rightSetpoint = rightRotationTrapezoidProfile.calculate(0.02);
    // Divide setpoint velocity by 10 so it's per 100ms
    double leftDriveRotationsPerSecond = Conversions.falconToRotationsPerSecond(leftSetpoint.velocity / 10);
    double rightDriveRotationsPerSecond = Conversions.falconToRotationsPerSecond(rightSetpoint.velocity / 10);
    mLeftFront.set(ControlMode.Position, leftSetpoint.position, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(leftDriveRotationsPerSecond, (leftDriveRotationsPerSecond - mPrevLeftDriveRotationsPerSecond) / dt));
    mRightFront.set(ControlMode.Position, rightSetpoint.position, DemandType.ArbitraryFeedForward, mDriveFeedforward.calculate(rightDriveRotationsPerSecond, (rightDriveRotationsPerSecond - mPrevRightDriveRotationsPerSecond) / dt));

    mPrevTime = curTime;
    mPrevLeftDriveRotationsPerSecond = leftDriveRotationsPerSecond;
    mPrevRightDriveRotationsPerSecond = rightDriveRotationsPerSecond;
  }

  public void stopDrive() {
    mLeftFront.stopMotor();
    mRightFront.stopMotor();
  }
}
