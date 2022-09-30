// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.util;

/** Add your docs here. */
public class JoystickToOutput {
    private JoystickToOutput() {};

    // https://www.desmos.com/calculator/4dkyeczdx6
    public static double calculateLinear(double joystick, double deadzone, double maxOutput) {
        if (joystick < -deadzone) {
        return maxOutput / (1 - deadzone) * (joystick + deadzone);
        } else if (joystick > deadzone) {
        return maxOutput / (1 - deadzone) * (joystick - deadzone);
        } else {
        return 0;
        }
    }
}
