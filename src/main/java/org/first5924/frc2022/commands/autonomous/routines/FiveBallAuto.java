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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  Timer mTimer = new Timer();
  Trajectory mFiveBallA = PathPlanner.loadPath("5 Ball Auto A", 2.5, 2);
  Trajectory mFiveBallB = PathPlanner.loadPath("5 Ball Auto B", 2.5, 2);
  Trajectory mFiveBallC = PathPlanner.loadPath("5 Ball Auto C", 2.5, 2);
  Trajectory mFiveBallD = PathPlanner.loadPath("5 Ball Auto D", 2.5, 2, true);

  /** Creates a new FiveBallAuto. */
  public FiveBallAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        driveSubsystem.setOdometryToPose(mFiveBallA.getInitialPose());
      }),
      new RamseteCommand(
        mFiveBallA,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        driveSubsystem::driveMPS,
        driveSubsystem),
      new WaitCommand(1),
      new RamseteCommand(
        mFiveBallB,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        driveSubsystem::driveMPS,
        driveSubsystem),
      new WaitCommand(0.5),
      new RamseteCommand(
        mFiveBallC,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        driveSubsystem::driveMPS,
        driveSubsystem),
      new RamseteCommand(
        mFiveBallD,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        driveSubsystem::driveMPS,
        driveSubsystem)
    );
  }
}
