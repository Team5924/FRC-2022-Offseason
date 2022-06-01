// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double sensorUnits;
  // Contains both magnitude and direction
  private final double velocity;

  private double endingPosition;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem driveSubsystem, double inches, double velocity) {
    m_drive = driveSubsystem;
    /**
     * Given inches, convert into rotations, then sensor units
     * rotations = inches / circumference
     * sensor units = rotations * 2048 * 9
     */
    this.sensorUnits = inches / DriveConstants.WHEEL_CIRCUMFERENCE * 2048 * 9;
    this.velocity = velocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endingPosition = m_drive.getLeftPosition() + sensorUnits;

    SmartDashboard.putNumber("Ending Position", endingPosition);
    SmartDashboard.putNumber("Speed", velocity);
    SmartDashboard.putNumber("Sensor Units", sensorUnits);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sensorUnits >= 0) {
      m_drive.tankDrive(velocity, velocity);
    } else {
      m_drive.tankDrive(-velocity, -velocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopLeft();
    m_drive.stopRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command drives straight, so only one side needs to be monitored for ending distance
    if (sensorUnits >= 0) {
      return m_drive.getLeftPosition() >= endingPosition;
    } else {
      return m_drive.getLeftPosition() <= endingPosition;
    }
  }
}
