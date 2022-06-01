// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;

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
