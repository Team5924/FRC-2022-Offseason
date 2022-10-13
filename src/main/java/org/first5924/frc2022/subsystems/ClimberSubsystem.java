// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first5924.frc2022.constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax mLeftClimberSpark = new CANSparkMax(ClimberConstants.kLeftClimberSparkPort, MotorType.kBrushed);
  private CANSparkMax mRightClimberSpark = new CANSparkMax(ClimberConstants.kRightClimberSparkPort, MotorType.kBrushed);

  /** Creates a new LeftClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeftClimber(double percent) {
    mLeftClimberSpark.set(percent);
  }

  public void setRightClimber(double percent) {
    mRightClimberSpark.set(percent);
  }
}
