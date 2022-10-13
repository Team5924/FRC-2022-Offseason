// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.shooter;

import org.first5924.frc2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopShooter extends CommandBase {
  private final ShooterSubsystem mShooter;

  /** Creates a new StopShooter. */
  public StopShooter(ShooterSubsystem shooterSubsystem) {
    mShooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.stopShooter();
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
