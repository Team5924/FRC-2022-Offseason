// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.shooter;

import org.first5924.frc2022.subsystems.ConveyorSubsystem;
import org.first5924.frc2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterEject extends CommandBase {
  private final ShooterSubsystem mShooter;
  private final ConveyorSubsystem mConveyor;

  /** Creates a new ShooterEject. */
  public ShooterEject(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    mShooter = shooterSubsystem;
    mConveyor = conveyorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooter, mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mConveyor.runConveyor(0.65);
    mConveyor.runFeeder(0.65);
    mShooter.set(0.35);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mConveyor.runConveyor(0);
    mConveyor.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
