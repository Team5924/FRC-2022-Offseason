// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.turret;

import org.first5924.frc2022.states.TurretState;
import org.first5924.frc2022.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualDisableTurret extends CommandBase {
  private final TurretSubsystem mTurret;
  private boolean mFinished = false;

  /** Creates a new ManualDisableTurret. */
  public ManualDisableTurret(TurretSubsystem turretSubsystem) {
    mTurret = turretSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mTurret.getState().equals(TurretState.DISABLED)) {
      mTurret.setState(TurretState.WAITING);
      mFinished = true;
    } else {
      mTurret.setVoltage(0);
      mTurret.setState(TurretState.WAITING);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFinished;
  }
}
