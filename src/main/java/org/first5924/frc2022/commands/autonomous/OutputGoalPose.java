// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OutputGoalPose extends CommandBase {
  private Timer mTimer = new Timer();
  private Trajectory mTrajectory;

  /** Creates a new OutputGoalPose. */
  public OutputGoalPose(Trajectory trajectory) {
    mTrajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Goal X", mTrajectory.sample(mTimer.get()).poseMeters.getX());
    SmartDashboard.putNumber("Goal Y", mTrajectory.sample(mTimer.get()).poseMeters.getY());
    SmartDashboard.putNumber("Goal Rotation", mTrajectory.sample(mTimer.get()).poseMeters.getRotation().getDegrees());
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
