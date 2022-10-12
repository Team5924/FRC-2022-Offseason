// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.intake;

import org.first5924.frc2022.states.IntakeState;
import org.first5924.frc2022.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployIntake extends InstantCommand {
  private final IntakeSubsystem mIntake;

  public DeployIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mIntake.getState() == IntakeState.RETRACTED) {
      mIntake.setState(IntakeState.DEPLOYING);
    }
  }
}
