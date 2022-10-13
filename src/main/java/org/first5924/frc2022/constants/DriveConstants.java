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

    public static final int kLeftCANCoder = 10;
    public static final int kRightCANCoder = 11;

    // In units of rotations
    public static final double ks = 0.57486;
    public static final double kv = 0.97304;
    public static final double ka = 0.065879;

    public static final double kP = 0.08;
    public static final double kI = 0;
    public static final double kD = 0;

    // For robot rotating
    public static final double kRotateP = 0.24;
    public static final double kRotateI = 0;
    public static final double kRotateD = 0.008;
    public static final double kRotatePositionTolerance = 1.25;
    public static final double kRotateVelocityTolerance = 0.5;



    // For robot rotating, in falcon units per second
    public static final double kRotateMaxSpeed = 5;
    public static final double kRotateMaxAccel = 5;

    public static final double kTrackwidth = 0.65292;
    public static final double kWheelCircumference = Units.inchesToMeters(4.18604 * Math.PI);
    public static final double kGearboxRatio = 9.04;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
