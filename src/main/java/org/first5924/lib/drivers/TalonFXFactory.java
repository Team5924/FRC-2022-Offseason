// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class TalonFXFactory {
    public static WPI_TalonFX createDefaultTalon(int id) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 10;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = 43;
        config.supplyCurrLimit.triggerThresholdCurrent = 57.5;
        config.supplyCurrLimit.triggerThresholdTime = 1.5;

        WPI_TalonFX talon = new WPI_TalonFX(id);
        talon.configAllSettings(config);
        talon.set(ControlMode.PercentOutput, 0);

        // Temporary, check if voltage compensation is being enabled
        System.out.println(talon.isVoltageCompensationEnabled() ? "Voltage compensation is enabled for talon with id " + id : "Voltage compensation is not enabled for talon with id " + id);

        return talon;
    }
}
