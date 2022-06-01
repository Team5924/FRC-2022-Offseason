// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.CONVEYOR_SPARK, MotorType.kBrushless);
  private CANSparkMax m_rollerSpark = new CANSparkMax(ConveyorConstants.ROLLER_SPARK, MotorType.kBrushless);

  private DigitalInput m_upperBeamBreak = new DigitalInput(ConveyorConstants.UPPER_BEAM_BREAK);
  private DigitalInput m_lowerBeamBreak = new DigitalInput(ConveyorConstants.LOWER_BEAM_BREAK);

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Beam Break", isUpperBeamBroken());
    SmartDashboard.putBoolean("Lower Beam Break", isLowerBeamBroken());
  }

  public void runRollers() {
    m_rollerSpark.set(0.3);
  }

  public void stopRollers() {
    m_rollerSpark.stopMotor();
  }

  public void run() {
    m_conveyorSpark.set(0.2);
  }

  public void stop() {
    m_conveyorSpark.stopMotor();
  }

  public void feedBallToShooter() {
    m_conveyorSpark.set(0.5);
  }

  public boolean isUpperBeamBroken() {
    return !m_upperBeamBreak.get();
  }

  public boolean isLowerBeamBroken() {
    return !m_lowerBeamBreak.get();
  }
}
