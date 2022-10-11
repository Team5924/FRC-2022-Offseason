// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.intake;

import org.first5924.frc2022.subsystems.IntakeSubsystem;

import org.first5924.frc2022.states.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployIntake extends CommandBase {
  private final IntakeSubsystem mIntake;

  /** Creates a new ToggleIntake. */
  public DeployIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intakeSubsystem;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (mIntake.getState()) {
      case RETRACTED:
        mIntake.runIntakeMotor(1);
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setState(IntakeState.DEPLOYED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Kill intake when current spikes past 5 or more amps
    if (mIntake.getCurrent() >= 5) {
      return true;
    } else {
      return false;
    }
  }
}
