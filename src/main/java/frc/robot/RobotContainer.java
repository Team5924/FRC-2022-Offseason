// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Eject;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.Rotate;
import frc.robot.commands.auto.routines.LeftDoubleBallAuto;
import frc.robot.commands.auto.routines.RightDoubleBallAuto;
import frc.robot.commands.auto.routines.SingleBallAuto;
import frc.robot.commands.auto.routines.TripleBallAuto;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  public static final DriveSubsystem m_drivetrain = new DriveSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final ClimberSubsystem m_climber = new ClimberSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER);

  XboxController m_operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  JoystickButton driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  JoystickButton driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  JoystickButton driverRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  JoystickButton operatorA = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  JoystickButton operatorB = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  JoystickButton operatorY = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  JoystickButton operatorX = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  JoystickButton operatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);

  // Declaring sendableObject for Autonomous here
  private final Command m_singleBallAuto = new SingleBallAuto(m_shooter, m_drivetrain, m_conveyor);
  private final Command m_leftDoubleBallAuto = new LeftDoubleBallAuto(m_shooter, m_conveyor, m_drivetrain, m_intake);
  private final Command m_rightDoubleBallAuto = new RightDoubleBallAuto(m_shooter, m_conveyor, m_drivetrain, m_intake);
  private final Command m_tripleBallAuto = new TripleBallAuto(m_shooter, m_conveyor, m_drivetrain, m_intake);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_intake.register();
    m_conveyor.register();
    m_shooter.register();
    m_drivetrain.register();
    m_climber.register();

    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_driverController::getLeftY, m_driverController::getRightY));
    m_conveyor.setDefaultCommand(new RunConveyor(m_conveyor, m_intake));

    // Configure the button bindings
    configureButtonBindings();

    m_autoChooser.setDefaultOption("Single Ball Auto", m_singleBallAuto);
    m_autoChooser.addOption("Left Double Ball Auto", m_leftDoubleBallAuto);
    m_autoChooser.addOption("Right Double Ball Auto", m_rightDoubleBallAuto);
    m_autoChooser.addOption("Triple Ball Auto", m_tripleBallAuto);

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
    driverRightBumper.whenPressed(new ToggleIntake(m_intake));

    operatorA.whenHeld(new Eject(m_conveyor, m_shooter));
    operatorB.whenPressed(new DriveDistance(m_drivetrain, 12, 0.1));
    //operatorB.whenPressed(new Rotate(m_drivetrain, 90, 0.1, true));
    operatorY.whenPressed(new ToggleShooter(m_shooter));
    operatorLeftBumper.whenHeld(new RetractClimber(m_climber));
    operatorRightBumper.whenHeld(new ExtendClimber(m_climber));
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