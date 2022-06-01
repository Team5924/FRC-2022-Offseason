// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.Rotate;
import frc.robot.commands.auto.WaitTime;
import frc.robot.subsystems.ConveyorSubsystem;

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
