// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class DriveConstants {
    public static final int kLeftFrontDrive = 4;
    public static final int kLeftBackDrive = 3;
    public static final int kRightFrontDrive = 2;
    public static final int kRightBackDrive = 1;

    // Max velocity that the drivetrain can do in sensor units per 100ms
    public static final double kMaxSpeed = 21776.25;

    public static final double ks = 0.5589;
    public static final double kv = 1.846;
    public static final double ka = 0.10965;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTrackwidthMeters = Units.inchesToMeters(0);
    public static final double kWheelCircumference = 0;
    public static final double kGearboxRatio = 0;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
