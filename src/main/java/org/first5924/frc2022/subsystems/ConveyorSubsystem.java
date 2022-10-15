// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first5924.frc2022.constants.ConveyorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
  private CANSparkMax mConveyor = new CANSparkMax(ConveyorConstants.kConveyorSparkPort, MotorType.kBrushless);
  private CANSparkMax mFeeder = new CANSparkMax(ConveyorConstants.kFeederSparkPort, MotorType.kBrushless);

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runConveyor(double percent) {
    mConveyor.set(percent);
  }

  public void runFeeder(double percent) {
    mFeeder.set(percent);
  }
}
