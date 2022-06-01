// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends CommandBase {
  private final ConveyorSubsystem m_conveyor;
  private final ShooterSubsystem m_shooter;

  // In ms, how long the conveyor should wait to feed after the shooter reaches speed
  private long shootDelayAfterAtSpeed = 100;
  // In ms, How long the command should last
  private long shootTime = 200;

  private long feedBallAt = -1;
  private long stopShootingAt;

  /** Creates a new AutoShoot. */
  public AutoShoot(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem) {
    m_conveyor = conveyorSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() >= feedBallAt) {
      // After 3 seconds, the vertical conveyor will feed the ball into the shooter
      m_conveyor.feedBallToShooter();
    }
    // If the time to feed the ball has not been set yet
    if (feedBallAt == -1) {
      if (m_shooter.isAtSpeed()) {
        feedBallAt = System.currentTimeMillis() + shootDelayAfterAtSpeed;
        stopShootingAt = feedBallAt + shootTime;
      }
    } else {
      if (System.currentTimeMillis() >= feedBallAt) {
        m_conveyor.feedBallToShooter();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= stopShootingAt;
  }
}