// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;
        // Percentage in every direction that you can push joystick with no response on drivetrain
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.08;
    }
    public static final class DriveConstants {
        // Max velocity that the drivetrain can do in sensor units per 100ms
        public static final double kMaxSpeed = 21776.25;

        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;

        public static final double kP = 0.16;
        public static final double kI = 0;
        public static final double kD = 1.6;

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

    public static final class ShooterConstants {
        /*
            Refer to NEO spreadsheet for measurements
        */
        public static final int LEADER_SHOOTER_SPARK = 9;
        public static final int FOLLOWER_SHOOTER_SPARK = 10;

        public static final int TIMEOUT_MS = 30;
        public static final int PID_LOOP_IDX = 0;
        public static final double SHOOTER_SPEED = 3500;
        public static final double FF = 0.000186;
        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 0;

        public static final double EJECT_SPEED = 1200;
        public static final double EJECT_FF = 0.000186;
        public static final double EJECT_P = 0.0001;
        public static final double EJECT_I = 0;
        public static final double EJECT_D = 0;


        public static final double ACCEPTABLE_RPM_ERROR = 200;
    }

    public static final class ConveyorConstants {
        public static final int ROLLER_SPARK = 7;
        public static final int CONVEYOR_SPARK = 8;
        public static final int UPPER_BEAM_BREAK = 0;
        public static final int LOWER_BEAM_BREAK = 2;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_SPARK = 6;
        public static final int CTRE_PCM = 5;
        public static final int LEFT_PNEUMATIC_FORWARD = 5;
        public static final int LEFT_PNEUMATIC_REVERSE = 2;
        public static final int RIGHT_PNEUMATIC_FORWARD = 4;
        public static final int RIGHT_PNEUMATIC_REVERSE = 3;

        public static final double INTAKE_RUN_SPEED = 0.5;
    }
    public static final class ClimberConstants {
        public static final int LEADER_CLIMBER_SPARK = 11;
        public static final int FOLLOWER_CLIMBER_SPARK = 12;
    }
}
