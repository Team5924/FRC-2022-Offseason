// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.first5924.frc2022.commands.autonomous.routines.FiveBallAuto;
import org.first5924.frc2022.commands.drive.CurvatureDrive;
import org.first5924.frc2022.commands.turret.TurretTrackTarget;
import org.first5924.frc2022.constants.OIConstants;
import org.first5924.frc2022.subsystems.DriveSubsystem;
import org.first5924.frc2022.subsystems.LimelightSubsystem;
import org.first5924.frc2022.subsystems.TurretSubsystem;
import org.first5924.frc2022.subsystems.IntakeSubsystem;

import org.first5924.frc2022.commands.intake.DeployIntake;
import org.first5924.frc2022.commands.intake.Eject;
import org.first5924.frc2022.commands.intake.RunIntake;
import org.first5924.frc2022.commands.intake.RetractIntake;

import org.first5924.frc2022.commands.autonomous.routines.OneBallAuto;
import org.first5924.frc2022.commands.autonomous.routines.TwoBallDefensiveAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // SUBSYSTEMS & COMMANDS
  private final DriveSubsystem mDrive = new DriveSubsystem();
  private final TurretSubsystem mTurret = new TurretSubsystem();
  private final LimelightSubsystem mLimelight = new LimelightSubsystem();
  private final IntakeSubsystem mIntake = new IntakeSubsystem();

  // CONTROLLER & BUTTONS
  private final XboxController mDriverController = new XboxController(OIConstants.kDriverController);

  private final JoystickButton mDriverLeftBumper = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mDriverB = new JoystickButton(mDriverController, XboxController.Button.kB.value);
  private final JoystickButton mDriverA = new JoystickButton(mDriverController, XboxController.Button.kA.value);
  private final JoystickButton mDriverX = new JoystickButton(mDriverController, XboxController.Button.kX.value);
  private final JoystickButton mDriverY = new JoystickButton(mDriverController, XboxController.Button.kY.value);

  // private final XboxController mOperatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  // SENDABLECHOOSER COMMANDS
  SendableChooser<Command> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    mDrive.register();
    mTurret.register();
    mLimelight.register();
    mIntake.register();

    // Default Commmands
    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    mTurret.setDefaultCommand(new TurretTrackTarget(mTurret, mLimelight));
    //mConveyor.setDefaultCommand(new RunConveyor(mConveyor, mIntake));
    mIntake.setDefaultCommand(new RunIntake(mIntake));

    // Configure the button bindings
    configureButtonBindings();

    // Auto
    mAutoChooser.setDefaultOption("5 Ball Auto", new FiveBallAuto(mDrive));
    mAutoChooser.addOption("2 Ball Defensive Auto", new TwoBallDefensiveAuto(mDrive));
    mAutoChooser.addOption("1 Ball Auto", new OneBallAuto(mDrive));
    SmartDashboard.putData(mAutoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // mDriverLeftBumper.whenHeld(new TurnInPlace(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    // mDriverB.whileHeld(new InstantCommand(() -> mTurret.setVoltage(1)));
    // mDriverB.whenReleased(new InstantCommand((() -> mTurret.setVoltage(0))));
    // mDriverA.whileHeld(new InstantCommand(() -> mTurret.setVoltage(-1)));
    // mDriverA.whenReleased(new InstantCommand(() -> mTurret.setVoltage(0)));
    // mDriverX.whenPressed(new InstantCommand(mTurret::zeroTurret));

    mDriverA.whenPressed(new DeployIntake(mIntake));
    mDriverB.whenPressed(new RetractIntake(mIntake));
    mDriverY.whenHeld(new Eject(mIntake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return mAutoChooser.getSelected();
  }
}