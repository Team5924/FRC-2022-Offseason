// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
  private final ConveyorSubsystem m_conveyor;
  private final ShooterSubsystem m_shooter;

  private boolean isAtSpeed = false;
  private boolean isPreparingToDisable = false;

  private int disableDelayTime = 500;

  private long disableVerticalConveyorAt = 0;

  private int commandLength = 500;

  private long endCommandAt;

  /** Creates a new Shoot. */
  public Shoot(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem) {
    m_conveyor = conveyorSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommandAt = System.currentTimeMillis() + commandLength;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isAtSpeed) {
      m_conveyor.feedBallToShooter();
      if (!m_shooter.isAtSpeed()) {
        isPreparingToDisable = true;
        isAtSpeed = false;
        disableVerticalConveyorAt = System.currentTimeMillis() + disableDelayTime;
      }
    } else if (isPreparingToDisable) {
      if (m_shooter.isAtSpeed()) {
        isPreparingToDisable = false;
        isAtSpeed = true;
      } else if (System.currentTimeMillis() >= disableVerticalConveyorAt) {
        m_conveyor.stop();
        isPreparingToDisable = false;
      }
    } else {
      if (m_shooter.isAtSpeed()) {
        isAtSpeed = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endCommandAt;
  }
}
