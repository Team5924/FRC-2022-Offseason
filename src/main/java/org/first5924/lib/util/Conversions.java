package org.first5924.lib.util;

import edu.wpi.first.math.util.Units;

public class Conversions {
    private Conversions() {};

    public static double metersToSensorUnits(double robotMeters, double wheelCircumferenceInches) {
        double rotations = robotMeters / Units.inchesToMeters(wheelCircumferenceInches);
        double sensorUnits = rotations * 2048;
        return sensorUnits;
    }

    public static double sensorUnitsToMeters(double sensorUnits, double wheelCircumferenceInches) {
        double rotations = sensorUnits / 2048;
        double meters = rotations * Units.inchesToMeters(wheelCircumferenceInches);
        return meters;
    }

    public static double RPMToFalcon(double RPM) {
        double sensorUnitsPerMinute = RPM * 2048;
        double falcon = sensorUnitsPerMinute / 600.0;
        return falcon;
    }

    public static double falconToRPM(double falcon) {
        double sensorUnitsPerMinute = falcon * 600;
        double RPM = sensorUnitsPerMinute / 2048;
        return RPM;
    }

    public static double MPSToFalcon(double MPS, double wheelCircumferenceInches) {
        double sensorUnitsPerSecond = metersToSensorUnits(MPS, wheelCircumferenceInches);
        double falcon = sensorUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToMPS(double falcon, double wheelCircumferenceInches) {
        double sensorUnitsPerSecond = falcon * 10;
        double MPS = sensorUnitsToMeters(sensorUnitsPerSecond, wheelCircumferenceInches);
        return MPS;
    }

    public static double RPMToRotationsPerSecond(double RPM) {
        double rotationsPerSecond = RPM / 60;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToRPM(double rotationsPerSecond) {
        double rotationsPerMinute = rotationsPerSecond * 60;
        return rotationsPerMinute;
    }

    public static double rotationsPerSecondToFalcon(double rotationsPerSecond) {
       double sensorUnitsPerSecond = rotationsPerSecond * 2048;
       double falcon = sensorUnitsPerSecond / 10;
       return falcon;
    }

    public static double falconToRotationsPerSecond(double falcon) {
        double sensorUnitsPerSecond = falcon * 10;
        double rotationsPerSecond = sensorUnitsPerSecond / 2048;
        return rotationsPerSecond;
    }

    public static double MPSToRotationsPerSecond(double MPS, double wheelCircumferenceInches) {
        double rotationsPerSecond = MPS / wheelCircumferenceInches;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToMPS(double rotationsPerSecond, double wheelCircumferenceInches) {
        double MPS = rotationsPerSecond * wheelCircumferenceInches;
        return MPS;
    }
}
