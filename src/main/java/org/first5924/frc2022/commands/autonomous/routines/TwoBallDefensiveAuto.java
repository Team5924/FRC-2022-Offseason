// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.autonomous.routines;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.first5924.frc2022.commands.autonomous.RotateToDegrees;
import org.first5924.frc2022.constants.DriveConstants;
import org.first5924.frc2022.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallDefensiveAuto extends SequentialCommandGroup {
  // Using Five Ball A path for first path
  Trajectory mFiveBallA = PathPlanner.loadPath("5 Ball Auto A", 2.5, 2);
  Trajectory mTwoBallDefensiveB = PathPlanner.loadPath("2 Ball Defensive Auto B", 2.5, 2);

  /** Creates a new FiveBallAuto. */
  public TwoBallDefensiveAuto(DriveSubsystem driveSubsystem) {
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
      new InstantCommand(driveSubsystem::stopDrive),
      new WaitCommand(1),
      new RotateToDegrees(-163, driveSubsystem),
      new InstantCommand(() -> driveSubsystem.setOdometryToPose(mTwoBallDefensiveB.getInitialPose())),
      new RamseteCommand(
        mTwoBallDefensiveB,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics,
        driveSubsystem::driveMPS,
        driveSubsystem),
      new InstantCommand(driveSubsystem::stopDrive),
      new RotateToDegrees(-85, driveSubsystem)
    );
  }
}
