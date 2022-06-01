// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDrive extends CommandBase {

  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_leftJoystickY;
  private final DoubleSupplier m_rightJoystickY;

  /** Creates a new TankDrive. */
  public TankDrive(DriveSubsystem subsystem, DoubleSupplier leftJoystickY, DoubleSupplier rightJoystickY) {
    m_drive = subsystem;
    // Method to get left joystick Y (-1 for fully forward, 1 for fully backward)
    m_leftJoystickY = leftJoystickY;
    // Method to get right joystick Y (-1 for fully forward, 1 for fully backward)
    m_rightJoystickY = rightJoystickY;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Negative signs to flip from negative for forward and positive for backward to positive for forward and negative for backward
    SmartDashboard.putNumber("Left Joy", -m_leftJoystickY.getAsDouble());
    SmartDashboard.putNumber("Right Joy", -m_rightJoystickY.getAsDouble());
    m_drive.tankSquaredDrive(-m_leftJoystickY.getAsDouble(), -m_rightJoystickY.getAsDouble());
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
