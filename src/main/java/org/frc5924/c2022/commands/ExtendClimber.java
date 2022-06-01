// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.commands;

import org.frc5924.c2022.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase {
  private final ClimberSubsystem m_climber;

  /** Creates a new ExtendClimber. */
  public ExtendClimber(ClimberSubsystem subsystem) {
    m_climber = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
