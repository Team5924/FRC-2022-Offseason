// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleShooter extends InstantCommand {
  private final ShooterSubsystem m_shooter;

  public ToggleShooter(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shooter.isRunning()) {
      m_shooter.stop();
    } else {
      m_shooter.run();
    }
  }
}
