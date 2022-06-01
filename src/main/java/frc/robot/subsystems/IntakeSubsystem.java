package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid; // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DoubleSolenoid.html
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final Compressor m_compressor = new Compressor(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid m_leftDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.LEFT_PNEUMATIC_FORWARD, IntakeConstants.LEFT_PNEUMATIC_REVERSE);
    private final DoubleSolenoid m_rightDoubleSolenoid = new DoubleSolenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.RIGHT_PNEUMATIC_FORWARD, IntakeConstants.RIGHT_PNEUMATIC_REVERSE);

    private final CANSparkMax m_intakeSpark = new CANSparkMax(IntakeConstants.INTAKE_SPARK, MotorType.kBrushless);

    public IntakeSubsystem() {
        //m_compressor.disable();

        m_leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

        m_intakeSpark.restoreFactoryDefaults();
        m_intakeSpark.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Left Pneumatic State", solenoidStatusAsString(m_leftDoubleSolenoid));
        SmartDashboard.putString("Right Pneumatic State", solenoidStatusAsString(m_rightDoubleSolenoid));
        SmartDashboard.putBoolean("Compressor Status", m_compressor.enabled());
        SmartDashboard.putBoolean("Intake Deployed", isDeployed());
        SmartDashboard.putBoolean("Compressor Pressurized", m_compressor.getPressureSwitchValue());
    }

    private String solenoidStatusAsString(DoubleSolenoid solenoid) {
        DoubleSolenoid.Value solenoidState = solenoid.get();
        if (solenoidState.equals(DoubleSolenoid.Value.kForward)) {
            return "Forward";
        } else if (solenoidState.equals(DoubleSolenoid.Value.kReverse)) {
            return "Reverse";
        } else if (solenoidState.equals(DoubleSolenoid.Value.kOff)) {
            return "Off";
        } else {
            return "Error";
        }
    }

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    public void disableCompressor() {
        m_compressor.disable();
    }

    public boolean isCompressorOn() {
        return m_compressor.enabled();
    }

    public void deploy() {
        m_leftDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        m_rightDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        m_leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isDeployed() {
        // returns the status of the intake, used to determine whether to retract or deploy intake
        return m_leftDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward) && m_rightDoubleSolenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    public void setIntakeMotorForward() {
        m_intakeSpark.set(IntakeConstants.INTAKE_RUN_SPEED);
    }

    public void stopIntakeMotor() {
        m_intakeSpark.stopMotor();
    }
}
