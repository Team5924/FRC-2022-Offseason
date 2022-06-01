// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
  private final DriveSubsystem m_drivetrain;
  private double radians;
  private double speed;

  private double arcLength;
  private double rotations;
  private double rotationsInSensorUnits;

  private double startPos;

  /** Creates a new Rotate. */
  public Rotate(DriveSubsystem driveSubsystem, double degrees, double speed) {
    m_drivetrain = driveSubsystem;
    /**
     * Degrees will be converted into radians
     * input as degrees for human convienience
     */
    this.radians = degrees * (Math.PI / 180);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_drivetrain.getLeftVelocity();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Given degrees, rotate the robot to the specified angle.
     * Arc length = Degree * 28 in. (radius)
     * Rotations = Arc Length / 4pi (circumference)
     * Rotations in sensor units = Rotations * 2048
     */

    arcLength = radians * 14;
    rotations = arcLength / (DriveConstants.WHEEL_CIRCUMFERENCE * Math.PI);
    rotationsInSensorUnits = rotations * 2048;

    if (radians >= 0) {
      m_drivetrain.tankDrive(speed, -speed);
    } else {
      m_drivetrain.tankDrive(-speed, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /**
     * command will finish after the robot has rotated to the specific angle, or
     * explained differently, after the robot has traveled the arc length
     */
    return (Math.abs(m_drivetrain.getLeftPosition() - startPos) >= rotationsInSensorUnits);
  }
}
