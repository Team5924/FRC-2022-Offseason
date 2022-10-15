// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.shooter;

import org.first5924.frc2022.states.ShooterState;
import org.first5924.frc2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleShooter extends InstantCommand {
  private final ShooterSubsystem mShooter;

  public ToggleShooter(ShooterSubsystem shooterSubsystem) {
    mShooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mShooter.getState().equals(ShooterState.RUNNING)) {
      mShooter.setState(ShooterState.STOPPED);
    } else {
      mShooter.setState(ShooterState.RUNNING);
    }
  }
}
