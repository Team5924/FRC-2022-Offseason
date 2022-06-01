// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleIntake extends InstantCommand {
  private final IntakeSubsystem m_intake;

  public ToggleIntake(IntakeSubsystem subsystem) {
    m_intake = subsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.isDeployed()) {
      m_intake.stopIntakeMotor();
      m_intake.retract();
    } else {
      m_intake.setIntakeMotorForward();
      m_intake.deploy();
    }
  }
}
