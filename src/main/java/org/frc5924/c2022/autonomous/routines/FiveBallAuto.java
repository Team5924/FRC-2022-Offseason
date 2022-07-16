// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.autonomous.routines;

import com.pathplanner.lib.PathPlanner;

import org.frc5924.c2022.Constants.DriveConstants;
import org.frc5924.c2022.subsystems.DriveSubsystem;
import org.frc5924.lib.autonomous.TalonFXRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  public FiveBallAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TalonFXRamseteCommand(
        PathPlanner.loadPath("5 Ball Auto A", 3, 2),
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        driveSubsystem.getLeftLeader(),
        driveSubsystem.getRightLeader(),
        driveSubsystem),
      new WaitCommand(1),
      new TalonFXRamseteCommand(
        PathPlanner.loadPath("5 Ball Auto B", 3, 2),
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        driveSubsystem.getLeftLeader(),
        driveSubsystem.getRightLeader(),
        driveSubsystem),
      new WaitCommand(0.5),
      new TalonFXRamseteCommand(
        PathPlanner.loadPath("5 Ball Auto C", 3.5, 2.5),
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        driveSubsystem.getLeftLeader(),
        driveSubsystem.getRightLeader(),
        driveSubsystem),
      new TalonFXRamseteCommand(
        PathPlanner.loadPath("5 Ball Auto D", 0.3, 1.5, true),
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        driveSubsystem.getLeftLeader(),
        driveSubsystem.getRightLeader(),
        driveSubsystem),
      new TalonFXRamseteCommand(
        PathPlanner.loadPath("5 Ball Auto E", 3.5, 2.5, true),
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        driveSubsystem.getLeftLeader(),
        driveSubsystem.getRightLeader(),
        driveSubsystem),
      new WaitCommand(1)
    );
  }
}
