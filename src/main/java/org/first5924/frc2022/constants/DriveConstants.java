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

    // In units of rotations
    public static final double ks = 0.57486;
    public static final double kv = 0.97304;
    public static final double ka = 0.065879;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTrackwidthMeters = Units.inchesToMeters(25.87);
    public static final double kWheelCircumferenceInches = 4 * Math.PI;
    public static final double kGearboxRatio = 9.04;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
