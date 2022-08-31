package org.first5924.lib.util;

public class Conversions {
    private Conversions() {};

    public static double metersToSensorUnits(double robotMeters, double wheelCircumference) {
        double rotations = robotMeters / wheelCircumference;
        double sensorUnits = rotations * 2048;
        return sensorUnits;
    }

    public static double sensorUnitsToMeters(double sensorUnits, double wheelCircumference) {
        double rotations = sensorUnits / 2048;
        double meters = rotations * wheelCircumference;
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

    public static double MPSToFalcon(double MPS, double wheelCircumference) {
        double sensorUnitsPerSecond = metersToSensorUnits(MPS, wheelCircumference);
        double falcon = sensorUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToMPS(double falcon, double wheelCircumference) {
        double sensorUnitsPerSecond = falcon * 10;
        double MPS = sensorUnitsToMeters(sensorUnitsPerSecond, wheelCircumference);
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

    public static double MPSToRotationsPerSecond(double MPS, double wheelCircumference) {
        double rotationsPerSecond = MPS / wheelCircumference;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToMPS(double rotationsPerSecond, double wheelCircumference) {
        double MPS = rotationsPerSecond * wheelCircumference;
        return MPS;
    }
}
