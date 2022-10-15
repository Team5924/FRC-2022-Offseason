// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2022.robot;

import org.first5924.frc2022.commands.autonomous.routines.FiveBallAuto;
import org.first5924.frc2022.commands.drive.CurvatureDrive;
import org.first5924.frc2022.commands.drive.TurnInPlace;
import org.first5924.frc2022.constants.OIConstants;
import org.first5924.frc2022.states.ShooterState;
import org.first5924.frc2022.subsystems.ConveyorSubsystem;
import org.first5924.frc2022.subsystems.DriveSubsystem;
import org.first5924.frc2022.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.first5924.frc2022.commands.turret.TurretTrackTarget;
import org.first5924.frc2022.subsystems.LimelightSubsystem;
import org.first5924.frc2022.subsystems.TurretSubsystem;
import org.first5924.frc2022.subsystems.IntakeSubsystem;

import org.first5924.frc2022.commands.intake.DeployIntake;
import org.first5924.frc2022.commands.intake.Eject;
import org.first5924.frc2022.commands.intake.RunIntake;
import org.first5924.frc2022.commands.shooter.RunShooter;
import org.first5924.frc2022.commands.shooter.ShooterEject;
import org.first5924.frc2022.commands.shooter.ToggleShooter;
import org.first5924.frc2022.commands.intake.RetractIntake;

import org.first5924.frc2022.commands.autonomous.routines.OneBallAuto;
import org.first5924.frc2022.commands.autonomous.routines.ThreeBallAutoTurretless;
import org.first5924.frc2022.commands.autonomous.routines.TwoBallDefensiveAuto;
import org.first5924.frc2022.commands.autonomous.routines.TwoOrOneBallAutoTurretless;
import org.first5924.frc2022.commands.conveyor.FeedShooter;
import org.first5924.frc2022.commands.conveyor.ManualRunConveyor;

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
  private final ConveyorSubsystem mConveyor = new ConveyorSubsystem();
  private final ShooterSubsystem mShooter = new ShooterSubsystem();

  // CONTROLLER & BUTTONS
  private final XboxController mDriverController = new XboxController(OIConstants.kDriverController);
  private final XboxController mOperatorController = new XboxController(OIConstants.kOperatorController);

  private final JoystickButton mDriverLeftBumper = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mDriverRightBumper = new JoystickButton(mDriverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mDriverStart = new JoystickButton(mDriverController, XboxController.Button.kStart.value);
  private final JoystickButton mDriverA = new JoystickButton(mDriverController, XboxController.Button.kA.value);
  private final JoystickButton mDriverB = new JoystickButton(mDriverController, XboxController.Button.kB.value);
  private final JoystickButton mDriverX = new JoystickButton(mDriverController, XboxController.Button.kX.value);
  private final JoystickButton mDriverY = new JoystickButton(mDriverController, XboxController.Button.kY.value);

  private final JoystickButton mOperatorLeftBumper = new JoystickButton(mOperatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mOperatorRightBumper = new JoystickButton(mOperatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mOperatorStart = new JoystickButton(mOperatorController, XboxController.Button.kStart.value);
  private final JoystickButton mOperatorA = new JoystickButton(mOperatorController, XboxController.Button.kA.value);
  private final JoystickButton mOperatorB = new JoystickButton(mOperatorController, XboxController.Button.kB.value);
  private final JoystickButton mOperatorX = new JoystickButton(mOperatorController, XboxController.Button.kX.value);
  private final JoystickButton mOperatorY = new JoystickButton(mOperatorController, XboxController.Button.kY.value);



  // Autonomous commands
  SendableChooser<Command> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    mDrive.register();
    mTurret.register();
    mLimelight.register();
    mIntake.register();
    mConveyor.register();
    mShooter.register();

    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    //mTurret.setDefaultCommand(new TurretTrackTarget(mTurret, mLimelight));
    //mIntake.setDefaultCommand(new RunIntake(mIntake));
    mShooter.setDefaultCommand(new RunShooter(mShooter, mLimelight));

    configureButtonBindings();

    mAutoChooser.setDefaultOption("3 Ball Auto Turretless", new ThreeBallAutoTurretless(mDrive, mIntake, mLimelight, mConveyor));
    mAutoChooser.addOption("2 or 1 Ball Auto Turretless", new TwoOrOneBallAutoTurretless(mDrive, mIntake, mLimelight, mConveyor));
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
    mDriverA.whenHeld(new ManualRunConveyor(mConveyor));

    mOperatorLeftBumper.whenPressed(new RetractIntake(mIntake));
    mOperatorRightBumper.whenPressed(new DeployIntake(mIntake));
    mOperatorA.whenHeld(new Eject(mIntake));

    mOperatorY.whenHeld(new ManualRunConveyor(mConveyor));

    mOperatorX.whenHeld(new ShooterEject(mShooter, mConveyor));
    mOperatorB.whenHeld(new FeedShooter(mConveyor));

    mOperatorStart.whenPressed(new ConditionalCommand(new ToggleShooter(mShooter), new RunShooter(mShooter, mLimelight), () -> mShooter.getState().equals(ShooterState.RUNNING)));
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