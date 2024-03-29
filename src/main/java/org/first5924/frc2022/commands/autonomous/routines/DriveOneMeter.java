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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOneMeter extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  Trajectory driveOneMeter = PathPlanner.loadPath("DriveOneMeter", 1, 1);

  /** Creates a new FiveBallAuto. */
  public DriveOneMeter(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        driveSubsystem.setOdometryToPose(driveOneMeter.getInitialPose());
      }),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          driveOneMeter,
          driveSubsystem::getPose,
          new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
          DriveConstants.kDriveKinematics,
          driveSubsystem::driveMPS,
          driveSubsystem),
        new OutputGoalPose(driveOneMeter))
    );
  }
}
