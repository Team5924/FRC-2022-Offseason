// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.robot;

import org.first5924.frc2022.commands.autonomous.RotateToDegrees;
import org.first5924.frc2022.commands.autonomous.routines.DriveOneMeter;
import org.first5924.frc2022.commands.autonomous.routines.FiveBallAuto;
import org.first5924.frc2022.commands.autonomous.routines.FiveBallAutoPrint;
import org.first5924.frc2022.commands.drive.CurvatureDrive;
import org.first5924.frc2022.commands.drive.TurnInPlace;
import org.first5924.frc2022.constants.OIConstants;
import org.first5924.frc2022.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem mDrive = new DriveSubsystem();

  private final XboxController mDriverController = new XboxController(OIConstants.kDriverController);

  private final JoystickButton mDriverLeftBumper = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mDriverA = new JoystickButton(mDriverController, XboxController.Button.kA.value);

  // private final XboxController mOperatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  SendableChooser<Command> mAutoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mDrive.register();

    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    //mConveyor.setDefaultCommand(new RunConveyor(mConveyor, mIntake));

    // Configure the button bindings
    configureButtonBindings();

    mAutoChooser.setDefaultOption("5 Ball Auto", new FiveBallAuto(mDrive));
    mAutoChooser.addOption("Print 5 Ball Auto", new FiveBallAutoPrint(mDrive));
    mAutoChooser.addOption("Drive One Meter", new DriveOneMeter(mDrive));
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
    mDriverLeftBumper.whenHeld(new TurnInPlace(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    mDriverA.whenPressed(new RotateToDegrees(90, mDrive));
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