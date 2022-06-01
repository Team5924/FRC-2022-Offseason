// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class WaitTime extends CommandBase {
  private DriveSubsystem m_drive;

  private long time;
  private long stopAt;

  /** Creates a new WaitTime. */
  public WaitTime(DriveSubsystem driveSubsystem, long time) {
    m_drive = driveSubsystem;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopAt = System.currentTimeMillis() + time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.stopLeft();
    m_drive.stopRight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= stopAt;
  }
}
