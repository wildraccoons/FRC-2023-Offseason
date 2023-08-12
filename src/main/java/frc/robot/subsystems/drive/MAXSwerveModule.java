package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * A wrapper for a MAXSwerve Module (wheel).
 * Specific to the REV MAXSwerve Module built with
 * a NEO for a driving, a NEO 550 for turning, Spark
 * MAXs for control, and Through Bore Absolute Encoder
 * through the wheels' axis of yaw rotation.
 */
public class MAXSwerveModule implements Sendable {
    public enum ModuleLabel {
        A, B, C, D,
    }

    private final double m_angularOffset;
    private final ModuleLabel m_label;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;

    private final SparkMaxPIDController m_drivePID;
    private final SparkMaxPIDController m_turnPID;

    private SwerveModuleState targetState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. Specific to the REV MAXSwerve Module built
     * with a NEO for drive, a NEO 550 for turning, Spark MAXs for control,
     * and a Through Bore Absolute Encoder.
     * 
     * @param driveId The CAN Id of the drive motor (NEO)
     * @param turnId The CAN Id of the turning motor (NEO 550)
     * @param angularOffset Angular offset of the swerve module compared to the zero.
     */
    public MAXSwerveModule(int driveId, int turnId, ModuleLabel moduleLabel) {
        m_label = moduleLabel;

        m_driveMotor = new CANSparkMax(driveId, ModuleConstants.driveMotorType);
        m_turnMotor = new CANSparkMax(turnId, ModuleConstants.turnMotorType);

        // Factory reset so the Spark MAXs are in a known state before config.
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        // Get encoders for each motor.
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Setup PID controllers and link correct encoders as feedback devices.
        m_drivePID = m_driveMotor.getPIDController();
        m_turnPID = m_turnMotor.getPIDController();
        m_drivePID.setFeedbackDevice(m_driveEncoder);
        m_turnPID.setFeedbackDevice(m_turnEncoder);

        // Apply the position and velocity conversion factors for the drive encoder.
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderVelocityFactor);

        // Apply the position and velocity conversion factors for the turning encoder.
        m_turnEncoder.setPositionConversionFactor(ModuleConstants.turnEncoderPositionFactor);
        m_turnEncoder.setVelocityConversionFactor(ModuleConstants.turnEncoderVelocityFactor);
        m_turnEncoder.setInverted(ModuleConstants.encoderReversed);

        // Enable PID wrap around for the turning motor. This allows for the PID
        // controller to go through 0 to get to the setpoint.
        m_turnPID.setPositionPIDWrappingEnabled(true);
        m_turnPID.setPositionPIDWrappingMaxInput(ModuleConstants.turnEncoderPositionPIDMinInput);
        m_turnPID.setPositionPIDWrappingMaxInput(ModuleConstants.turnEncoderPositionPIDMaxInput);

        // Set the PID gains for the drive motor.
        m_drivePID.setP(ModuleConstants.driveKP);
        m_drivePID.setI(ModuleConstants.driveKI);
        m_drivePID.setD(ModuleConstants.driveKD);
        m_drivePID.setFF(ModuleConstants.driveFF);
        m_drivePID.setOutputRange(ModuleConstants.driveMinOutput, ModuleConstants.driveMaxOutput);

        // Set the PID gains for the turn motor.
        m_turnPID.setP(ModuleConstants.turnKP);
        m_turnPID.setI(ModuleConstants.turnKI);
        m_turnPID.setD(ModuleConstants.turnKD);
        m_turnPID.setFF(ModuleConstants.turnFF);
        m_turnPID.setOutputRange(ModuleConstants.turnMinOutput, ModuleConstants.turnMaxOutput);

        m_driveMotor.setIdleMode(ModuleConstants.driveMotorIdleMode);
        m_turnMotor.setIdleMode(ModuleConstants.turnMotorIdleMode);
        m_driveMotor.setSmartCurrentLimit(ModuleConstants.driveMotorCurrentLimit);
        m_turnMotor.setSmartCurrentLimit(ModuleConstants.turnMotorCurrentLimit);

        // Burn configuration to flash so that if a brown out occurs during
        // operation the Spark will maintain the same configuration.
        m_driveMotor.burnFlash();
        m_turnMotor.burnFlash();

        this.m_angularOffset = DriveConstants.angularOffsets[moduleLabel.ordinal()];
        targetState.angle = new Rotation2d(m_turnEncoder.getPosition());
        m_driveEncoder.setPosition(0);
    }

    /**
     * Returns the current position of the module.
     * 
     * @return The current position of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            new Rotation2d(m_turnEncoder.getPosition() - m_angularOffset)
        );
    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_turnEncoder.getPosition() - m_angularOffset)
        );
    }

    /**
     *  Sets the target state for the module.
     * 
     * @param target The target state with speed and angle.
     */
    public void setTargetState(SwerveModuleState target) {
        // Apply chassis angular offset to the target state
        SwerveModuleState correctedTarget = new SwerveModuleState();
        correctedTarget.speedMetersPerSecond = target.speedMetersPerSecond;
        correctedTarget.angle = target.angle.plus(Rotation2d.fromRadians(m_angularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedTarget = SwerveModuleState.optimize(correctedTarget, new Rotation2d(m_turnEncoder.getPosition()));
        
        // Command driving and turning Spark MAXs to their respective setpoints.
        m_drivePID.setReference(optimizedTarget.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turnPID.setReference(optimizedTarget.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        targetState = target;
    }

    /** Zeros the drive encoder. */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Module " + m_label.name() + " Angle (rads)", () -> m_turnEncoder.getPosition() - m_angularOffset, null);
        builder.addDoubleProperty("Module " + m_label.name() + " Distance (m)", () -> m_driveEncoder.getPosition(), null);
        builder.addDoubleProperty("Module " + m_label.name() + " Speed (m/s)", () -> m_driveEncoder.getVelocity(), null);
    }
}
