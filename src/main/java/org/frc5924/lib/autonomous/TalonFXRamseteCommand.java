// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.lib.autonomous;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5924.c2022.Constants.DriveConstants;
import org.frc5924.lib.util.Conversions;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TalonFXRamseteCommand extends CommandBase {
  // Uses logic from WPILib's RamseteCommand class but using CTRE's PID instead of WPILib's PID
  private final Timer mTimer = new Timer();
  private final Trajectory mTrajectory;
  private final Supplier<Pose2d> mPose;
  private final RamseteController mFollower;
  private final SimpleMotorFeedforward mFeedforward;
  private final DifferentialDriveKinematics mKinematics;
  private final WPI_TalonFX mLeftLeader;
  private final WPI_TalonFX mRightLeader;
  private DifferentialDriveWheelSpeeds mPrevSpeeds;
  private double mPrevTime;

  /** Creates a new TalonFXRamseteCommand. */
  public TalonFXRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, WPI_TalonFX leftLeader, WPI_TalonFX rightLeader, Subsystem... requirements) {
    mTrajectory = ErrorMessages.requireNonNullParam(trajectory, "trajectory", "TalonFXRamseteCommand");
    mPose = ErrorMessages.requireNonNullParam(pose, "pose", "TalonFXRamseteCommand");
    mFollower = ErrorMessages.requireNonNullParam(controller, "controller", "TalonFXRamseteCommand");
    mFeedforward = feedforward;
    mKinematics = ErrorMessages.requireNonNullParam(kinematics, "kinematics", "TalonFXRamseteCommand");
    mLeftLeader = ErrorMessages.requireNonNullParam(leftLeader, "leftLeader", "TalonFXRamseteCommand");
    mRightLeader = ErrorMessages.requireNonNullParam(rightLeader, "rightLeader", "TalonFXRamseteCommand");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mPrevTime = -1;
    var initialState = mTrajectory.sample(0);
    mPrevSpeeds =
        mKinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    mTimer.reset();
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = mTimer.get();
    double dt = curTime - mPrevTime;

    if (mPrevTime < 0) {
      mLeftLeader.set(0);
      mRightLeader.set(0);
      mPrevTime = curTime;
      return;
    }

    var targetWheelSpeeds =
        mKinematics.toWheelSpeeds(
            mFollower.calculate(mPose.get(), mTrajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    mLeftLeader.set(
      ControlMode.Velocity,
      Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
        leftSpeedSetpoint,
        DriveConstants.kWheelCircumference,
        DriveConstants.kGearboxRatio),
      DemandType.ArbitraryFeedForward,
      mFeedforward.calculate(
        Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
          leftSpeedSetpoint,
          DriveConstants.kWheelCircumference,
          DriveConstants.kGearboxRatio),
        Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
          (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt,
          DriveConstants.kWheelCircumference,
          DriveConstants.kGearboxRatio)));

    mRightLeader.set(
      ControlMode.Velocity,
      Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
        rightSpeedSetpoint,
        DriveConstants.kWheelCircumference,
        DriveConstants.kGearboxRatio),
      DemandType.ArbitraryFeedForward,
      mFeedforward.calculate(
        Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
          rightSpeedSetpoint,
          DriveConstants.kWheelCircumference,
          DriveConstants.kGearboxRatio),
        Conversions.robotMetersPerSecondToFalconUnitsPer100ms(
          (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt,
          DriveConstants.kWheelCircumference,
          DriveConstants.kGearboxRatio)));

    mPrevSpeeds = targetWheelSpeeds;
    mPrevTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTimer.stop();

    if (interrupted) {
      mLeftLeader.set(0);
      mRightLeader.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds());
  }
}
