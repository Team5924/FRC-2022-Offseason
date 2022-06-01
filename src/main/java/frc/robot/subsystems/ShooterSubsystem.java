package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ShooterConstants;
public class ShooterSubsystem extends SubsystemBase {

    /**
     * The shooter has two motors spinning, relative to each other,
     * in the opposite dierction. The leaderSparkMax is also the
     * PIDController, and the other one follows.
     */

    private CANSparkMax m_leaderShooterSpark = new CANSparkMax(ShooterConstants.LEADER_SHOOTER_SPARK, MotorType.kBrushless);
    private CANSparkMax m_followerShooterSpark = new CANSparkMax(ShooterConstants.FOLLOWER_SHOOTER_SPARK, MotorType.kBrushless);

    private SparkMaxPIDController m_PIDController;

    // Subject to change
    private RelativeEncoder m_encoder;

    // Shuffleboard NeteworkTable
    private NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
    private NetworkTable m_table = m_inst.getTable("SmartDashboard");

    // TEMP USE - MARKED FOR DELETION
    private double shooterSetpoint = ShooterConstants.SHOOTER_SPEED;
    private double shooterFF = ShooterConstants.FF;

    // Constructor for ShooterSubsystem class
    public ShooterSubsystem() {
        m_leaderShooterSpark.restoreFactoryDefaults();
        m_followerShooterSpark.restoreFactoryDefaults();

        m_leaderShooterSpark.setInverted(true);
        m_followerShooterSpark.follow(m_leaderShooterSpark, true);

        m_PIDController = m_leaderShooterSpark.getPIDController();

        m_encoder = m_leaderShooterSpark.getEncoder();

        // TEMP REMOVAL
        // m_PIDController.setFF(ShooterConstants.FF, 0);
        m_PIDController.setP(ShooterConstants.P, 0);
        m_PIDController.setI(ShooterConstants.I, 0);
        m_PIDController.setD(ShooterConstants.D, 0);

        m_PIDController.setFF(ShooterConstants.EJECT_FF, 1);
        m_PIDController.setP(ShooterConstants.EJECT_P, 1);
        m_PIDController.setI(ShooterConstants.EJECT_I, 1);
        m_PIDController.setD(ShooterConstants.EJECT_D, 1);

        SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);
        SmartDashboard.putNumber("Shooter FF", shooterFF);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);
        SmartDashboard.putNumber("Shooter Speed", m_encoder.getVelocity());
        SmartDashboard.putBoolean("Shooter At Speed", isAtSpeed());
        SmartDashboard.putBoolean("Shooter Running", isRunning());

        // Retreiving data from Shuffleboard live
        shooterSetpoint = m_table.getEntry("Shooter Setpoint").getValue().getDouble();
        shooterFF = m_table.getEntry("Shooter FF").getValue().getDouble();

        m_PIDController.setFF(shooterFF, 0);
    }

    // Checks to see if shooter is ready to fire
    public boolean isAtSpeed() {

        return Math.abs(m_encoder.getVelocity() - shooterSetpoint) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
        // return Math.abs(m_encoder.getVelocity() - ShooterConstants.SHOOTER_SPEED) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
    }

    public boolean isRunning() {
        return Math.abs(m_encoder.getVelocity()) > 1;
    }

    public void eject() {
        // speed is in RPM
        m_PIDController.setReference(ShooterConstants.EJECT_SPEED, CANSparkMax.ControlType.kVelocity, 1);
    }

    public void run() {
        // speed is in RPM
        m_PIDController.setReference(shooterSetpoint, CANSparkMax.ControlType.kVelocity, 0);
    }


    public void stop() {
        m_leaderShooterSpark.stopMotor();
    }
}
