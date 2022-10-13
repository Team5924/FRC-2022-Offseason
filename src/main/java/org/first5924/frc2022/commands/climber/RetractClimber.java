// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.climber;

import java.util.function.DoubleSupplier;

import org.first5924.frc2022.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimber extends CommandBase {
  private final ClimberSubsystem mClimber;

  private final DoubleSupplier mRightJoystickX;

  /** Creates a new ExtendClimber. */
  public RetractClimber(ClimberSubsystem climberSubsystem, DoubleSupplier rightJoystickX) {
    mClimber = climberSubsystem;
    mRightJoystickX = rightJoystickX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mRightJoystickX.getAsDouble() < -0.3) {
      mClimber.setLeftClimber(-1);
    } else if (mRightJoystickX.getAsDouble() > 0.3) {
      mClimber.setRightClimber(-1);
    } else {
      mClimber.setLeftClimber(-1);
      mClimber.setRightClimber(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.setLeftClimber(0);
    mClimber.setRightClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
