// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.commands.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.first5924.frc2022.states.IntakeState;
import org.first5924.frc2022.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {
  private final IntakeSubsystem mIntake;

  // For fluttering the motor on & off
  private final Timer timer = new Timer();
  private final double delay = 0.5;

  /** Creates a new RunIntake. */
  public RunIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intakeSubsystem;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (mIntake.getState()) {
      case DEPLOYED:
        mIntake.runIntake(0.75);
        timer.start();
        if (timer.hasElapsed(delay)) {
          mIntake.stopIntake();
          timer.stop();
          timer.reset();
        }
        // mIntake.runIntakeWheels(0.65);
        break;
      case RETRACTED:
        mIntake.setNeutralMode(NeutralMode.Brake);
        mIntake.runIntake(-0.8);
        timer.start();
        if (timer.hasElapsed(delay)) {
          mIntake.stopIntake();
          timer.stop();
          timer.reset();
        }
        break;
      case DEPLOYING:
        mIntake.setNeutralMode(NeutralMode.Coast);
        mIntake.runIntake(1.75);
        if (mIntake.getIntakeCurrent() >= 35) {
          mIntake.setState(IntakeState.DEPLOYED);
        }
        break;
      case RETRACTING:
        mIntake.stopWheels();
        mIntake.runIntake(-1.75);
        if (mIntake.getIntakeCurrent() >= 35) {
          mIntake.setState(IntakeState.RETRACTED);
        }
        break;
      case EJECTING:
        mIntake.runWheels(-1);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
