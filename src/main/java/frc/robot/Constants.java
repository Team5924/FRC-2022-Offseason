// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        // Motor IDs
        public static final int LEFT_FRONT_TALON = 4;
        public static final int RIGHT_FRONT_TALON = 2;
        public static final int LEFT_BACK_TALON = 3;
        public static final int RIGHT_BACK_TALON = 1;

        // Max velocity that the drivetrain can do in sensor units per 100ms
        public static final double MAX_VELOCITY = 21776.25;
        // Percentage to limit drivetrain to (of above number)
        public static final double PERCENT_MAX_VELOCITY_LIMIT = 0.8;

        public static final int TIMEOUT_MS = 30;
        public static final int PID_LOOP_IDX = 0;
        public static final int SLOT_IDX = 0;
        public static final double F = 0.75 * 1023 / (MAX_VELOCITY * PERCENT_MAX_VELOCITY_LIMIT);
        public static final double P = 0.16;
        public static final double I = 0;
        public static final double D = 1.6;

        // Current limit for motors in amps
        public static final double CURRENT_LIMIT = 40;
        // Amps at which to trigger current limit
        public static final double TRIGGER_THRESHOLD_CURRENT = 45;
        // Trigger threshold must be attained for this amount of time to trigger current limit
        public static final double TRIGGER_THRESHOLD_TIME = 1;
        public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;

        public static final double AUTO_DRIVE_SPEED = 0.4;
        public static final double AUTO_ROTATE_SPEED = 0.15;
        public static final double AUTO_INTAKE_BALL_DRIVE_SPEED = 0.1;
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
