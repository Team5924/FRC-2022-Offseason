// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.shooter;

import org.first5924.frc2022.states.ShooterState;
import org.first5924.frc2022.subsystems.LimelightSubsystem;
import org.first5924.frc2022.subsystems.ShooterSubsystem;
import org.first5924.lib.util.LinearInterpolationTable;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooter extends CommandBase {
  private final ShooterSubsystem mShooter;
  private final LimelightSubsystem mLimelight;

  private LinearInterpolationTable mVerticalOffsetToShooterSpeedTable = new LinearInterpolationTable();

  /** Creates a new RunShooter. */
  public RunShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    mShooter = shooterSubsystem;
    mLimelight = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);

    // mVerticalOffsetToShooterSpeedTable.add();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // mShooter.setShooterSpeed(mVerticalOffsetToShooterSpeedTable.interpolate(mLimelight.getCrosshairVerticalOffset()));
    if (mShooter.getState().equals(ShooterState.RUNNING)) {
      mShooter.set(0.6);
    } else {
      mShooter.set(0);
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
