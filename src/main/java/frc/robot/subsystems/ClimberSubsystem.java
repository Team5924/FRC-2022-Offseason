// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax m_leaderClimberSpark = new CANSparkMax(ClimberConstants.LEADER_CLIMBER_SPARK, MotorType.kBrushed);
  CANSparkMax m_followerClimberSpark = new CANSparkMax(ClimberConstants.FOLLOWER_CLIMBER_SPARK, MotorType.kBrushed);
  RelativeEncoder m_leaderClimberEncoder = m_leaderClimberSpark.getEncoder();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_followerClimberSpark.follow(m_leaderClimberSpark);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", m_leaderClimberEncoder.getPosition());
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
