package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Telemetry;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * A wrapper for a MAXSwerve Module (wheel).
 * Specific to the REV MAXSwerve Module built with
 * a NEO for a driving, a NEO 550 for turning, Spark
 * MAXs for control, and Through Bore Absolute Encoder
 * through the wheels' axis of yaw rotation.
 */
public class MAXSwerveModule {
    public enum ModuleLabel {
        A,
        B,
        C,
        D,
    }

    private final double angularOffset;
    private final ModuleLabel m_label;

    private final CANSparkMax driveSpark;
    private final CANSparkMax turnSpark;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkMaxPIDController drivePID;
    private final SparkMaxPIDController turnPID;

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

        driveSpark = new CANSparkMax(driveId, ModuleConstants.driveMotorType);
        turnSpark = new CANSparkMax(turnId, ModuleConstants.turnMotorType);

        // Factory reset so the Spark MAXs are in a known state before config.
        driveSpark.restoreFactoryDefaults();
        turnSpark.restoreFactoryDefaults();

        // Get encoders for each motor.
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getAbsoluteEncoder(Type.kDutyCycle);

        // Setup PID controllers and link correct encoders as feedback devices.
        drivePID = driveSpark.getPIDController();
        turnPID = turnSpark.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        // Apply the position and velocity conversion factors for the drive encoder.
        driveEncoder.setPositionConversionFactor(ModuleConstants.driveEncoderPositionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.driveEncoderVelocityFactor);

        // Apply the position and velocity conversion factors for the turning encoder.
        turnEncoder.setPositionConversionFactor(ModuleConstants.turnEncoderPositionFactor);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.turnEncoderVelocityFactor);
        turnEncoder.setInverted(ModuleConstants.encoderReversed);

        // Enable PID wrap around for the turning motor. This allows for the PID
        // controller to go through 0 to get to the setpoint.
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMaxInput(ModuleConstants.turnEncoderPositionPIDMinInput);
        turnPID.setPositionPIDWrappingMaxInput(ModuleConstants.turnEncoderPositionPIDMaxInput);

        // Set the PID gains for the drive motor.
        drivePID.setP(ModuleConstants.driveKP);
        drivePID.setI(ModuleConstants.driveKI);
        drivePID.setD(ModuleConstants.driveKD);
        drivePID.setFF(ModuleConstants.driveFF);
        drivePID.setOutputRange(ModuleConstants.driveMinOutput, ModuleConstants.driveMaxOutput);

        // Set the PID gains for the turn motor.
        turnPID.setP(ModuleConstants.turnKP);
        turnPID.setI(ModuleConstants.turnKI);
        turnPID.setD(ModuleConstants.turnKD);
        turnPID.setFF(ModuleConstants.turnFF);
        turnPID.setOutputRange(ModuleConstants.turnMinOutput, ModuleConstants.turnMaxOutput);

        driveSpark.setIdleMode(ModuleConstants.driveMotorIdleMode);
        turnSpark.setIdleMode(ModuleConstants.turnMotorIdleMode);
        driveSpark.setSmartCurrentLimit(ModuleConstants.driveMotorCurrentLimit);
        turnSpark.setSmartCurrentLimit(ModuleConstants.turnMotorCurrentLimit);

        // Burn configuration to flash so that if a brown out occurs during
        // operation the Spark will maintain the same configuration.
        driveSpark.burnFlash();
        turnSpark.burnFlash();

        this.angularOffset = DriveConstants.angularOffsets[moduleLabel.ordinal()];
        targetState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    /**
     * Returns the current position of the module.
     * 
     * @return The current position of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
        );
    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
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
        correctedTarget.angle = target.angle.plus(Rotation2d.fromRadians(angularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedTarget = SwerveModuleState.optimize(correctedTarget, new Rotation2d(turnEncoder.getPosition()));
        
        // Command driving and turning Spark MAXs to their respective setpoints.
        drivePID.setReference(optimizedTarget.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPID.setReference(optimizedTarget.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        targetState = target;
    }

    /** Zeros the drive encoder. */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void outputTelemetry() {
        SwerveModulePosition position = getPosition();

        SmartDashboard.putNumber("Module " + m_label.name() + " Angle (rads)", position.angle.getRadians());
        SmartDashboard.putNumber("Module " + m_label.name() + " Distance (m)", position.distanceMeters);
        SmartDashboard.putNumber("Module " + m_label.name() + " Speed (m/s)", driveEncoder.getVelocity());
    }
}
