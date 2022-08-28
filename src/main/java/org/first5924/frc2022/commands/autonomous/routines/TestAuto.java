// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.autonomous.routines;

import com.pathplanner.lib.PathPlanner;

import org.first5924.frc2022.constants.DriveConstants;
import org.first5924.frc2022.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  Trajectory testPath = PathPlanner.loadPath("Test Path", 0.25, 0.25);

  /** Creates a new TestAuto. */
  public TestAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(driveSubsystem::zeroYaw),
      new InstantCommand(driveSubsystem::)
      new InstantCommand(mTimer::start),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          testPath,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new InstantCommand(() -> {
          SmartDashboard.putNumber("Goal X", testPath.sample(mTimer.get()).poseMeters.getX());
          SmartDashboard.putNumber("Goal Y", testPath.sample(mTimer.get()).poseMeters.getY());
          SmartDashboard.putNumber("Goal Rotation", testPath.sample(mTimer.get()).poseMeters.getRotation().getDegrees());
        })
      )
    );
  }
}
