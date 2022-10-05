// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first5924.frc2022.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax mIntakeSpark = new CANSparkMax(IntakeConstants.kIntakeSparkID, MotorType.kBrushless);

  // Troubleshoot variables - TEMP
  private boolean status;
  private boolean forward;
  private boolean backward;


  private PowerDistribution mPDM = new PowerDistribution(0, ModuleType.kCTRE);
  private double totalCurrent = mPDM.getTotalCurrent();
  private double intakeCurrent = mPDM.getCurrent(IntakeConstants.kIntakeChannel);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Status", status);
    SmartDashboard.putBoolean("Forward", forward);
    SmartDashboard.putBoolean("Backward", backward);

    System.out.println(totalCurrent);
  }

  public void deploy(double speed) {
    mIntakeSpark.setVoltage(speed);
    status = true; forward = true; backward = false;
  }

  public void retract(double speed) {
    mIntakeSpark.setVoltage(-speed);
    status = true; forward = false; backward = true;
  }

  public void stop() {
    mIntakeSpark.stopMotor();
    status = false; forward = false; backward = false;
  }
}
