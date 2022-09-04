// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.autonomous.routines;

import com.pathplanner.lib.PathPlanner;

import org.first5924.frc2022.commands.autonomous.OutputGoalPose;
import org.first5924.frc2022.constants.DriveConstants;
import org.first5924.frc2022.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAutoPrint extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  Trajectory fiveBallA = PathPlanner.loadPath("5 Ball Auto A", 1, 1);
  Trajectory fiveBallB = PathPlanner.loadPath("5 Ball Auto B", 1, 1);
  Trajectory fiveBallC = PathPlanner.loadPath("5 Ball Auto C", 1, 1);
  Trajectory fiveBallD = PathPlanner.loadPath("5 Ball Auto D", 1, 1, true);

  /** Creates a new FiveBallAuto. */
  public FiveBallAutoPrint(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        driveSubsystem.setOdometryToPose(fiveBallA.getInitialPose());
      }),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          fiveBallA,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new OutputGoalPose(fiveBallA)),
      new InstantCommand(driveSubsystem::stopDrive),
      new WaitCommand(1),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          fiveBallB,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new OutputGoalPose(fiveBallB)),
      new InstantCommand(driveSubsystem::stopDrive),
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          fiveBallC,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new OutputGoalPose(fiveBallC)),
      new InstantCommand(driveSubsystem::stopDrive),
      new WaitCommand(0.25),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          fiveBallD,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new OutputGoalPose(fiveBallD))
    );
  }
}
