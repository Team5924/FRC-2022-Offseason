// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.commands.auto.routines;

import org.frc5924.c2022.Constants.DriveConstants;
import org.frc5924.c2022.commands.ToggleIntake;
import org.frc5924.c2022.commands.ToggleShooter;
import org.frc5924.c2022.commands.auto.AutoShoot;
import org.frc5924.c2022.commands.auto.DriveDistance;
import org.frc5924.c2022.commands.auto.Rotate;
import org.frc5924.c2022.commands.auto.WaitTime;
import org.frc5924.c2022.subsystems.ConveyorSubsystem;
import org.frc5924.c2022.subsystems.DriveSubsystem;
import org.frc5924.c2022.subsystems.IntakeSubsystem;
import org.frc5924.c2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TripleBallAuto extends SequentialCommandGroup {
  private ShooterSubsystem m_shooter;
  private DriveSubsystem m_drivetrain;
  private ConveyorSubsystem m_conveyor;
  private IntakeSubsystem m_intake;

  /** Creates a new TripleBallAuto. */
  public TripleBallAuto(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_conveyor = conveyorSubsystem;
    m_drivetrain = driveSubsystem;
    m_intake = intakeSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleShooter(m_shooter),
      new ToggleIntake(m_intake),
      new AutoShoot(m_conveyor, m_shooter),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, 26.34, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new Rotate(m_drivetrain, -22.5, DriveConstants.AUTO_ROTATE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, (71.47 - 15), DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, (-71.47 + 15), DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new Rotate(m_drivetrain, 22.5, DriveConstants.AUTO_ROTATE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, -27.34, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new AutoShoot(m_conveyor, m_shooter),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, 67, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new Rotate(m_drivetrain, 95.754, DriveConstants.AUTO_ROTATE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, 85.43, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, -85.43, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new Rotate(m_drivetrain, -95.754, DriveConstants.AUTO_ROTATE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new DriveDistance(m_drivetrain, -68, DriveConstants.AUTO_DRIVE_SPEED),
      new WaitTime(m_drivetrain, 100),
      new AutoShoot(m_conveyor, m_shooter)
    );
  }
}
