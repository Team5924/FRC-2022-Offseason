// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class TalonSRXFactory {
    private TalonSRXFactory() {};

    public static WPI_TalonSRX createDefaultTalon(int id) {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.voltageCompSaturation = 10;
        config.continuousCurrentLimit = 43;
        config.peakCurrentLimit = 57;
        config.peakCurrentDuration = 1500;

        WPI_TalonSRX talon = new WPI_TalonSRX(id);
        talon.configAllSettings(config);
        talon.enableVoltageCompensation(true);
        talon.set(ControlMode.PercentOutput, 0);

        return talon;
    }
}
