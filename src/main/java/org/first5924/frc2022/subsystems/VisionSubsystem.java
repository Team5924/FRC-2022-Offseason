// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private UsbCamera driverCamera = CameraServer.startAutomaticCapture("Driver Camera", 0);
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    driverCamera.setFPS(20);
    driverCamera.setBrightness(70);
    driverCamera.setResolution(256, 192);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
