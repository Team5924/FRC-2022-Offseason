package org.frc5924.c2022.util;

import edu.wpi.first.math.util.Units;

public class Conversions {
    public static double robotMetersPerSecondToFalconUnitsPer100ms(double metersPerSecond, double wheelCircumferenceInches, double gearboxRatio) {
        double metersPer100ms = metersPerSecond / 10;
        double wheelRotationsPer100ms = metersPer100ms / Units.inchesToMeters(wheelCircumferenceInches);
        double motorRotationsPer100ms = wheelRotationsPer100ms * gearboxRatio;
        double falconUnitsPer100ms = motorRotationsPer100ms * 4096;
        return falconUnitsPer100ms;
    }

    public static double falconUnitsPer100msToRobotMetersPerSecond(double falconUnitsPer100ms, double wheelCircumferenceInches, double gearboxRatio) {
        double motorRotationsPer100ms = falconUnitsPer100ms / 4096;
        double wheelRotationsPer100ms = motorRotationsPer100ms / gearboxRatio;
        double metersPer100ms = wheelRotationsPer100ms * Units.inchesToMeters(wheelCircumferenceInches);
        double metersPerSecond = metersPer100ms * 10;
        return metersPerSecond;
    }

    public static double sensorUnitsToMeters(double sensorUnits, double wheelCircumferenceInches) {
        return sensorUnits * 4096 * Units.inchesToMeters(wheelCircumferenceInches);
    }
}
