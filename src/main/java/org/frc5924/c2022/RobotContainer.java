// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5924.c2022;

import org.frc5924.c2022.Constants.OIConstants;
import org.frc5924.c2022.commands.Eject;
import org.frc5924.c2022.commands.ExtendClimber;
import org.frc5924.c2022.commands.RetractClimber;
import org.frc5924.c2022.commands.RunConveyor;
import org.frc5924.c2022.commands.ToggleIntake;
import org.frc5924.c2022.commands.ToggleShooter;
import org.frc5924.c2022.commands.drive.CurvatureDrive;
import org.frc5924.c2022.commands.drive.TurnInPlace;
import org.frc5924.c2022.subsystems.ClimberSubsystem;
import org.frc5924.c2022.subsystems.ConveyorSubsystem;
import org.frc5924.c2022.subsystems.DriveSubsystem;
import org.frc5924.c2022.subsystems.IntakeSubsystem;
import org.frc5924.c2022.subsystems.ShooterSubsystem;

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
  private final IntakeSubsystem mIntake = new IntakeSubsystem();
  private final ConveyorSubsystem mConveyor = new ConveyorSubsystem();
  private final ShooterSubsystem mShooter = new ShooterSubsystem();
  private final ClimberSubsystem mClimber = new ClimberSubsystem();

  private final XboxController mDriverController = new XboxController(OIConstants.DRIVER_CONTROLLER);

  private final JoystickButton mDriverLeftBumper = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mDriverRightBumper = new JoystickButton(mDriverController, XboxController.Button.kRightBumper.value);

  private final XboxController mOperatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  private final JoystickButton mOperatorA = new JoystickButton(mOperatorController, XboxController.Button.kA.value);
  private final JoystickButton mOperatorY = new JoystickButton(mOperatorController, XboxController.Button.kY.value);
  private final JoystickButton mOperatorLeftBumper = new JoystickButton(mOperatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mOperatorRightBumper = new JoystickButton(mOperatorController, XboxController.Button.kRightBumper.value);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mIntake.register();
    mConveyor.register();
    mShooter.register();
    mDrive.register();
    mClimber.register();

    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightY));
    //mConveyor.setDefaultCommand(new RunConveyor(mConveyor, mIntake));

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(m_autoChooser);
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
    mDriverRightBumper.whenPressed(new ToggleIntake(mIntake));

    mOperatorA.whenHeld(new Eject(mConveyor, mShooter));
    mOperatorY.whenPressed(new ToggleShooter(mShooter));
    mOperatorLeftBumper.whenHeld(new RetractClimber(mClimber));
    mOperatorRightBumper.whenHeld(new ExtendClimber(mClimber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}