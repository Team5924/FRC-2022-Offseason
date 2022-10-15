// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.first5924.frc2022.states.TurretState;
import org.first5924.frc2022.subsystems.LimelightSubsystem;
import org.first5924.frc2022.subsystems.TurretSubsystem;

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
          double angle = mTurret.getTurretPosition() - mLimelight.getCrosshairHorizontalOffset();
          mTurret.turnTurretToDegrees(angle);
        } else {
          mTurret.setState(TurretState.WAITING);
        }
        break;
      case WAITING:
        mTurret.turnTurretToDegrees(0);
        if (mLimelight.isTargetDetected()) {
          mTurret.setState(TurretState.TRACKING);
        }
        break;
      case DISABLED:
        mTurret.setState(TurretState.WAITING);
        break;
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
