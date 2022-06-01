// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Eject extends CommandBase {
  private ConveyorSubsystem m_conveyor;
  private ShooterSubsystem m_shooter;

  private boolean wasShooterRunning;

  /** Creates a new Eject. */
  public Eject(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem) {
    m_conveyor = conveyorSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasShooterRunning = m_shooter.isRunning();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.eject();
    m_conveyor.feedBallToShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (wasShooterRunning) {
      m_shooter.run();
    } else {
      m_shooter.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
