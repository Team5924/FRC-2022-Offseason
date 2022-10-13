// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import org.first5924.frc2022.constants.RobotConstants;
import org.first5924.frc2022.constants.TurretConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Target Detected?", isTargetDetected());
    SmartDashboard.putNumber("Crosshair Horizontal Offset", getCrosshairHorizontalOffset());
  }

  public boolean isTargetDetected() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }

  public double getCrosshairHorizontalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getCrosshairVerticalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    if (getCrosshairVerticalOffset() == 0) {
      return 0;
    } else {
      // Unit: Inches
      return (RobotConstants.kGoalHeight - TurretConstants.kLimelightLensHeight) / Math.tan(Units.degreesToRadians(TurretConstants.kLimelightMountAngle + getCrosshairVerticalOffset()));
    }
  }
}
