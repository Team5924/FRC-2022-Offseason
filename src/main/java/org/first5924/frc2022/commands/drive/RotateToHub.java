// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.drive;

import org.first5924.frc2022.subsystems.DriveSubsystem;
import org.first5924.frc2022.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateToHub extends CommandBase {
  private final DriveSubsystem mDrive;
  private final LimelightSubsystem mLimelight;

  /** Creates a new RotateToHub. */
  public RotateToHub(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    mDrive = driveSubsystem;
    mLimelight = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive, mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mLimelight.isTargetDetected()) {
      mDrive.rotateToDegrees(mDrive.getOffsetRotation2d().getDegrees() - mLimelight.getCrosshairHorizontalOffset(), 3.5);
    } else {
      mDrive.turnInPlace(0, -0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(mLimelight.getCrosshairHorizontalOffset()) <= 1.1;
  }
}
