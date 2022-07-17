// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5924.c2022.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax m_leaderClimberSpark = new CANSparkMax(ClimberConstants.LEADER_CLIMBER_SPARK, MotorType.kBrushed);
  CANSparkMax m_followerClimberSpark = new CANSparkMax(ClimberConstants.FOLLOWER_CLIMBER_SPARK, MotorType.kBrushed);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_followerClimberSpark.follow(m_leaderClimberSpark);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    m_leaderClimberSpark.set(0.5);
  }

  public void retract() {
    m_leaderClimberSpark.set(-0.5);
  }

  public void stop() {
    m_leaderClimberSpark.stopMotor();
  }
}
