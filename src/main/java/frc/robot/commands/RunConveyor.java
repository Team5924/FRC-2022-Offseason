// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunConveyor extends CommandBase {
  private final ConveyorSubsystem m_conveyor;
  private final IntakeSubsystem m_intake;

  /** Creates a new RunConveyor. */
  public RunConveyor(ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    m_conveyor = conveyorSubsystem;
    m_intake = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isDeployed()) {
      if (!m_conveyor.isUpperBeamBroken()) {
        m_conveyor.run();
      } else {
        m_conveyor.stop();
      }
      if (!m_conveyor.isLowerBeamBroken()) {
        m_conveyor.runRollers();
      } else {
        m_conveyor.stopRollers();
      }
    } else {
      m_conveyor.stop();
      m_conveyor.stopRollers();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
