package org.first5924.lib.util;

import edu.wpi.first.math.util.Units;

public class Conversions {
    private Conversions() {};

    public static double robotMetersToSensorUnits(double robotMeters, double gearRatio, double wheelCircumferenceInches) {
        double wheelRotations = robotMeters / Units.inchesToMeters(wheelCircumferenceInches);
        double motorRotations = wheelRotations * gearRatio;
        double sensorUnits = motorRotations * 2048;
        return sensorUnits;
    }

    public static double sensorUnitsToRobotMeters(double sensorUnits, double gearRatio, double wheelCircumferenceInches) {
        double motorRotations = sensorUnits / 2048;
        double wheelRotations = motorRotations / gearRatio;
        double robotMeters = wheelRotations * Units.inchesToMeters(wheelCircumferenceInches);
        return robotMeters;
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorUnitsPerMinute = motorRPM * 2048;
        double falcon = sensorUnitsPerMinute / 600.0;
        return falcon;
    }

    public static double falconToRPM(double falcon, double gearRatio) {
        double sensorUnitsPerMinute = falcon * 600;
        double motorRPM = sensorUnitsPerMinute / 2048;
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double MPSToFalcon(double MPS, double gearRatio, double wheelCircumferenceInches) {
        double sensorUnitsPerSecond = robotMetersToSensorUnits(MPS, gearRatio, wheelCircumferenceInches);
        double falcon = sensorUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToMPS(double falcon, double gearRatio, double wheelCircumferenceInches) {
        double sensorUnitsPerSecond = falcon * 10;
        double MPS = sensorUnitsToRobotMeters(sensorUnitsPerSecond, gearRatio, wheelCircumferenceInches);
        return MPS;
    }
}
