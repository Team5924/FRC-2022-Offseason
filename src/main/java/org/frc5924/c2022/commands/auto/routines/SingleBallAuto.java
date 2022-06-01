// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.commands.auto.routines;

import org.frc5924.c2022.Constants.DriveConstants;
import org.frc5924.c2022.commands.ToggleShooter;
import org.frc5924.c2022.commands.auto.AutoShoot;
import org.frc5924.c2022.commands.auto.DriveDistance;
import org.frc5924.c2022.subsystems.ConveyorSubsystem;
import org.frc5924.c2022.subsystems.DriveSubsystem;
import org.frc5924.c2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleBallAuto extends SequentialCommandGroup {
  private ShooterSubsystem m_shooter;
  private DriveSubsystem m_drivetrain;
  private ConveyorSubsystem m_conveyor;

  /** Creates a new AutoAimAndShoot. */
  public SingleBallAuto(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, ConveyorSubsystem conveyorSubsystem) {
    m_shooter = shooterSubsystem;
    m_conveyor = conveyorSubsystem;
    m_drivetrain = driveSubsystem;

    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new ToggleShooter(m_shooter),
      new AutoShoot(m_conveyor, m_shooter),
      new DriveDistance(m_drivetrain, 90, DriveConstants.AUTO_DRIVE_SPEED)
    );
  }
}
