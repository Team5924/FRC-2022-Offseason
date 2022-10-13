// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.conveyor;

import org.first5924.frc2022.subsystems.ConveyorSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
  private final ConveyorSubsystem mConveyor;
  private final Timer mTimer = new Timer();

  /** Creates a new Shoot. */
  public FeedShooter(ConveyorSubsystem conveyorSubsystem) {
    mConveyor = conveyorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mConveyor.runFeeder(0.5);
    if (mTimer.get() > 0.3) {
      mConveyor.runConveyor(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mConveyor.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
