package org.first5924.lib.util;

public class Conversions {
    private Conversions() {};

    // Rotation conversions

    public static double degreesToFalconUnits(double degrees) {
        double falconUnits = degrees / 360 * 2048;
        return falconUnits;
    }

    public static double falconUnitsToDegrees(double falconUnits) {
        double degrees = falconUnits / 2048 * 360;
        return degrees;
    public static double rotationsToFalconUnits(double rotations) {
        double falconUnits = rotations * 2048;
        return falconUnits;
    }

    public static double falconUnitsToRotations(double falconUnits) {
        double rotations = falconUnits / 2048;
        return rotations;
    }

    // Rotation per time conversions

    public static double rotationsPerSecondToFalcon(double rotationsPerSecond) {
        double falconUnitsPerSecond = rotationsPerSecond * 2048;
        double falcon = falconUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToRotationsPerSecond(double falcon) {
        double falconUnitsPerSecond = falcon * 10;
        double rotationsPerSecond = falconUnitsPerSecond / 2048;
        return rotationsPerSecond;
    }

    public static double RPMToRotationsPerSecond(double RPM) {
        double rotationsPerSecond = RPM / 60;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToRPM(double rotationsPerSecond) {
        double rotationsPerMinute = rotationsPerSecond * 60;
        return rotationsPerMinute;
    }

    public static double RPMToFalcon(double RPM) {
        double falconUnitsPerMinute = RPM * 2048;
        double falcon = falconUnitsPerMinute / 600.0;
        return falcon;
    }

    public static double falconToRPM(double falcon) {
        double falconUnitsPerMinute = falcon * 600;
        double RPM = falconUnitsPerMinute / 2048;
        return RPM;
    }

    // Length to rotation / rotation to length conversions

    public static double metersToFalconUnits(double robotMeters, double wheelCircumference) {
        double rotations = robotMeters / wheelCircumference;
        double falconUnits = rotations * 2048;
        return falconUnits;
    }

    public static double falconUnitsToMeters(double falconUnits, double wheelCircumference) {
        double rotations = falconUnits / 2048;
        double meters = rotations * wheelCircumference;
        return meters;
    }

    public static double metersToDegrees(double robotMeters, double wheelCircumference) {
        double rotations = robotMeters / wheelCircumference;
        double degrees = rotations * 360;
        return degrees;
    }

    public static double degreesToMeters(double degrees, double wheelCircumference) {
        double rotations = degrees / 360;
        double meters = rotations * wheelCircumference;
        return meters;
    }

    public static double metersToRotations(double robotMeters, double wheelCircumference) {
        double rotations = robotMeters / wheelCircumference;
        return rotations;
    }

    public static double rotationsToMeters(double rotations, double wheelCircumference) {
        double meters = rotations * wheelCircumference;
        return meters;
    }

    // Length over time to rotation over time / rotation over time to length over time conversions

    public static double MPSToFalcon(double MPS, double wheelCircumference) {
        double falconUnitsPerSecond = metersToFalconUnits(MPS, wheelCircumference);
        double falcon = falconUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToMPS(double falcon, double wheelCircumference) {
        double falconUnitsPerSecond = falcon * 10;
        double MPS = falconUnitsToMeters(falconUnitsPerSecond, wheelCircumference);
        return MPS;
    }

    public static double MPSToRotationsPerSecond(double MPS, double wheelCircumference) {
        double rotationsPerSecond = MPS / wheelCircumference;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToMPS(double rotationsPerSecond, double wheelCircumference) {
        double MPS = rotationsPerSecond * wheelCircumference;
        return MPS;
    }

    public static double MPSToDegreesPerSecond(double MPS, double wheelCircumference) {
        double rotationsPerSecond = MPSToRotationsPerSecond(MPS, wheelCircumference);
        double degreesPerSecond = rotationsPerSecond * 360;
        return degreesPerSecond;
    }

    public static double degreesPerSecondToMPS(double degreesPerSecond, double wheelCircumference) {
        double rotationsPerSecond = degreesPerSecond / 360;
        double MPS = rotationsPerSecondToMPS(rotationsPerSecond, wheelCircumference);
        return MPS;
    }

    public static double degreesPerSecondToFalcon(double degreesPerSecond) {
        double falconUnitsPerSecond = degreesPerSecond / 360 * 2048;
        double falcon = falconUnitsPerSecond / 10;
        return falcon;
    }

    public static double falconToDegreesPerSecond(double falcon) {
        double falconUnitsPerSecond = falcon * 10;
        double degreesPerSecond = falconUnitsPerSecond / 2048 * 360;
        return degreesPerSecond;
    }
}
