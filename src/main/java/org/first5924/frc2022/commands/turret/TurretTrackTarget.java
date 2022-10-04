// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.turret;

import org.first5924.frc2022.constants.TurretConstants;
import org.first5924.frc2022.states.TurretState;
import org.first5924.frc2022.subsystems.LimelightSubsystem;
import org.first5924.frc2022.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretTrackTarget extends CommandBase {
  private final TurretSubsystem mTurret;
  private final LimelightSubsystem mLimelight;

  /** Creates a new TurretTrackTarget. */
  public TurretTrackTarget(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    mTurret = turretSubsystem;
    mLimelight = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(mTurret.getState()) {
      case TRACKING:
        if (mLimelight.isTargetDetected()) {
          double angle = mTurret.getTurretPosition() * 360 + mLimelight.getCrosshairHorizontalOffset();
          if (angle > TurretConstants.kRangeOfMotion / 2) {
            mTurret.setState(TurretState.SEARCHING_LEFT);
          } else if (angle < -TurretConstants.kRangeOfMotion / 2) {
            mTurret.setState(TurretState.SEARCHING_RIGHT);
          } else {
            mTurret.turnDegrees(mLimelight.getCrosshairHorizontalOffset());
          }
        } else if (mTurret.getTurretPosition() >= 0) {
          mTurret.setState(TurretState.SEARCHING_RIGHT);
        } else {
          mTurret.setState(TurretState.SEARCHING_LEFT);
        }
      case SEARCHING_RIGHT:
        mTurret.turnToDegrees(TurretConstants.kRangeOfMotion / 2);
        if (mLimelight.isTargetDetected()) {
          double angle = mTurret.getTurretPosition() * 360 + mLimelight.getCrosshairHorizontalOffset();
          if (angle >= -TurretConstants.kRangeOfMotion / 2 && angle <= TurretConstants.kRangeOfMotion / 2) {
            mTurret.setState(TurretState.TRACKING);
          }
        }
      case SEARCHING_LEFT:
        mTurret.turnToDegrees(-TurretConstants.kRangeOfMotion / 2);
        if (mLimelight.isTargetDetected()) {
          double angle = mTurret.getTurretPosition() * 360 + mLimelight.getCrosshairHorizontalOffset();
          if (angle >= -TurretConstants.kRangeOfMotion / 2 && angle <= TurretConstants.kRangeOfMotion / 2) {
            mTurret.setState(TurretState.TRACKING);
          }
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
