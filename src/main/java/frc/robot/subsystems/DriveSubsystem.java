// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftDTLeader = new WPI_TalonFX(DriveConstants.LEFT_FRONT_TALON);
  private final WPI_TalonFX m_rightDTLeader = new WPI_TalonFX(DriveConstants.RIGHT_FRONT_TALON);
  private final WPI_TalonFX m_leftDTFollower = new WPI_TalonFX(DriveConstants.LEFT_BACK_TALON);
  private final WPI_TalonFX m_rightDTFollower = new WPI_TalonFX(DriveConstants.RIGHT_BACK_TALON);

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    // Sychronizes motors with other motors on the same side of the robot
    m_leftDTFollower.follow(m_leftDTLeader);
    m_rightDTFollower.follow(m_rightDTLeader);
    m_rightDTLeader.setInverted(true);
    m_rightDTFollower.setInverted(InvertType.FollowMaster);
    // Configure talon settings
    configTalon(m_leftDTLeader);
    configTalon(m_rightDTLeader);
  }

  private void configTalon(WPI_TalonFX motorController) {
    // Factory default hardware to prevent unexpected behaviour
		motorController.configFactoryDefault();

		// Config neutral deadband to be the smallest possible
		motorController.configNeutralDeadband(0.001, DriveConstants.TIMEOUT_MS);

		// Config sensor used for Primary PID [Velocity]
    motorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DriveConstants.PID_LOOP_IDX, DriveConstants.TIMEOUT_MS);

    // Config allowable closed loop error
    motorController.configAllowableClosedloopError(0, 1, DriveConstants.TIMEOUT_MS);

		// Config the peak and nominal outputs
		motorController.configNominalOutputForward(0, DriveConstants.TIMEOUT_MS);
		motorController.configNominalOutputReverse(0, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputForward(1, DriveConstants.TIMEOUT_MS);
		motorController.configPeakOutputReverse(-1, DriveConstants.TIMEOUT_MS);

    // Config amp limits to avoid breaking a circuit
    motorController.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.CURRENT_LIMIT, DriveConstants.TRIGGER_THRESHOLD_CURRENT, DriveConstants.TRIGGER_THRESHOLD_TIME));

    // Config motors to brake when neutral (superior for drivetrain)
    motorController.setNeutralMode(NeutralMode.Brake);

		// Config the Velocity closed loop gains in slot0
		motorController.config_kF(DriveConstants.SLOT_IDX, DriveConstants.F, DriveConstants.TIMEOUT_MS);
		motorController.config_kP(DriveConstants.SLOT_IDX, DriveConstants.P, DriveConstants.TIMEOUT_MS);
		motorController.config_kI(DriveConstants.SLOT_IDX, DriveConstants.I, DriveConstants.TIMEOUT_MS);
		motorController.config_kD(DriveConstants.SLOT_IDX, DriveConstants.D, DriveConstants.TIMEOUT_MS);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 *
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity Feet", sensorToFeetPerSecond(getLeftVelocity()));
    SmartDashboard.putNumber("Right Velocity Feet", sensorToFeetPerSecond(getRightVelocity()));

    SmartDashboard.putNumber("Left Velocity",getLeftVelocity());
  }

  public double getLeftVelocity() {
    return m_leftDTLeader.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return m_rightDTLeader.getSelectedSensorVelocity();
  }

  public double getLeftPosition() {
    return m_leftDTFollower.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return m_rightDTFollower.getSelectedSensorPosition();
  }
  // Percent is a decimal, 0 <= percent <= 1
  public void tankDrive(double leftPercent, double rightPercent) {
    if (leftPercent < -DriveConstants.PERCENT_MAX_VELOCITY_LIMIT) {
      m_leftDTLeader.set(ControlMode.Velocity, -DriveConstants.PERCENT_MAX_VELOCITY_LIMIT);
    } else if (leftPercent > DriveConstants.PERCENT_MAX_VELOCITY_LIMIT) {
      m_leftDTLeader.set(ControlMode.Velocity, DriveConstants.PERCENT_MAX_VELOCITY_LIMIT);
    } else {
      m_leftDTLeader.set(ControlMode.Velocity, leftPercent * DriveConstants.MAX_VELOCITY);
    }
    if (rightPercent < -DriveConstants.PERCENT_MAX_VELOCITY_LIMIT) {
      m_rightDTLeader.set(ControlMode.Velocity, -DriveConstants.PERCENT_MAX_VELOCITY_LIMIT);
    } else if (rightPercent > DriveConstants.PERCENT_MAX_VELOCITY_LIMIT) {
      m_rightDTLeader.set(ControlMode.Velocity, DriveConstants.PERCENT_MAX_VELOCITY_LIMIT);
    } else {
      m_rightDTLeader.set(ControlMode.Velocity, rightPercent * DriveConstants.MAX_VELOCITY);
    }
  }

  // The math for this method is here
  // https://www.desmos.com/calculator/waxy7hdapv
  public void tankSquaredDrive(double leftJoystick, double rightJoystick) {
    double leftOutputPercent;
    double rightOutputPercent;
    if (leftJoystick < -OIConstants.DRIVER_CONTROLLER_DEADBAND) {
      leftOutputPercent = DriveConstants.PERCENT_MAX_VELOCITY_LIMIT / Math.pow(1 - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2) * -Math.pow(leftJoystick + OIConstants.DRIVER_CONTROLLER_DEADBAND, 2);
    } else if (leftJoystick <= OIConstants.DRIVER_CONTROLLER_DEADBAND) {
      leftOutputPercent = 0;
    } else {
      leftOutputPercent = DriveConstants.PERCENT_MAX_VELOCITY_LIMIT / Math.pow(1 - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2) * Math.pow(leftJoystick - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2);
    }
    if (rightJoystick < -OIConstants.DRIVER_CONTROLLER_DEADBAND) {
      rightOutputPercent = DriveConstants.PERCENT_MAX_VELOCITY_LIMIT / Math.pow(1 - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2) * -Math.pow(rightJoystick + OIConstants.DRIVER_CONTROLLER_DEADBAND, 2);
    } else if (rightJoystick <= OIConstants.DRIVER_CONTROLLER_DEADBAND) {
      rightOutputPercent = 0;
    } else {
      rightOutputPercent = DriveConstants.PERCENT_MAX_VELOCITY_LIMIT / Math.pow(1 - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2) * Math.pow(rightJoystick - OIConstants.DRIVER_CONTROLLER_DEADBAND, 2);
    }
    m_leftDTLeader.set(ControlMode.Velocity, leftOutputPercent * DriveConstants.MAX_VELOCITY);
    m_rightDTLeader.set(ControlMode.Velocity, rightOutputPercent * DriveConstants.MAX_VELOCITY);
  }

  public void stopLeft() {
    m_leftDTLeader.stopMotor();
  }

  public void stopRight() {
    m_rightDTLeader.stopMotor();
  }

  private double sensorToFeetPerSecond(double sensor) {
    /*
    Multiply by 10 to change to per second from per 100ms
    Divide by 2048 to get rotations from sensor units
    Multiply by 4π to get inches from rotations
    Divide by 12 to get feet from inches
    Divide by 9 to account for gearbox
    */
    return Math.round(sensor * 10 / 2048 * 4 * Math.PI / 12 / 9);
  }
}
