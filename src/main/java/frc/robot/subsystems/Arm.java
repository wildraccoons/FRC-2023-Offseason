package frc.robot.subsystems;

// Rev
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
// WPI
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// Local
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ArmConstants;
import wildlib.PIDSparkMax;
import wildlib.utils.MathUtils;

public class Arm extends SubsystemBase {
    private final PIDSparkMax m_extension;
    private final PIDSparkMax m_rotation;

    private static Arm m_instance;

    public static Arm getInstance() {
        if (m_instance == null) {
            m_instance = new Arm(
                new PIDSparkMax(
                    IOConstants.armExtensionId,
                    MotorType.kBrushless,
                    ArmConstants.kExtensionP,
                    ArmConstants.kExtensionI,
                    ArmConstants.kExtensionD
                ),
                new PIDSparkMax(
                    IOConstants.armRotationId,
                    MotorType.kBrushless,
                    ArmConstants.kRotationP,
                    ArmConstants.kRotationI,
                    ArmConstants.kRotationD
                )
            );
        }

        return m_instance;
    }

    private Arm(PIDSparkMax extension, PIDSparkMax rotation) {
        m_extension = extension;
        m_rotation = rotation;

        // Configure rotation
        m_rotation.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_rotation.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kMinRotation);

        m_rotation.setMinOutput(-0.25);
        m_rotation.setMaxOutput(0.4);

        m_rotation.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_rotation.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kMaxRotation);

        m_rotation.setInverted(true);

        m_rotation.setIdleMode(IdleMode.kBrake);

        m_rotation.setPositionConversionFactor(ArmConstants.kRotationConversionFactor);
        m_rotation.setVelocityConversionFactor(ArmConstants.kRotationConversionFactor);

        // Configure extension
        m_extension.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_extension.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kMaxExtension);

        m_extension.setMaxOutput(0.30);
        m_extension.setMinOutput(-0.30);

        m_extension.setInverted(true);

        m_extension.setIdleMode(IdleMode.kBrake);

        m_extension.setPositionConversionFactor(ArmConstants.kExtensionConversionFactor);
        m_extension.setVelocityConversionFactor(ArmConstants.kExtensionConversionFactor);
    }

    @Override
    public void periodic() {}
    
    /**
     * Sets the speed of the extension motor.
     * 
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setExtension(double speed) {
        m_extension.set(speed);
    }
    
    /**
     * Sets the reference position for the extension PID.
     * 
     * @param position Reference position to set.
     * @return {@link REVLibError#kOk} if successful.
     */
    public REVLibError setExtensionPosition(double position) {
        return m_extension.setReference(position, ControlType.kPosition);
    }

    public REVLibError setExtensionVelocity(double velocity) {
        return m_extension.setReference(velocity, ControlType.kVelocity);
    }
    
    public REVLibError holdExtension() {
        return setExtensionPosition(getExtension());
    }

    public Command setExtensionAndWait(double position, double range) {
        return new InstantCommand(() -> {
                System.out.println("extension called");
                setExtensionPosition(position);
            }, this)
            .andThen(new WaitUntilCommand(() -> {
                zeroLimit();
                return MathUtils.closeEnough(getExtension(), position, range/2);
            }));
    }

    public double getExtension() {
        return m_extension.getPosition();
    }
    
    public void setRotation(double speed) {
        m_rotation.set(speed);
    }

    public REVLibError setRotationPosition(double position) {
        return m_rotation.setReference(position, ControlType.kPosition);
    }

    public REVLibError setRotationVelocity(double velocity) {
        return m_rotation.setReference(velocity, ControlType.kVelocity);
    }

    public CommandBase setRotationAndWait(double position, double range) {
        return new InstantCommand(() -> {
                System.out.println("arm rotation called");
                setRotationPosition(position);
            }, this)
            .andThen(new WaitUntilCommand(() -> {
                return MathUtils.closeEnough(getRotation(), position, range/2);
            }));
    }

    public REVLibError holdRotation() {
        return setRotationPosition(getRotation());
    }

    public double getRotation() {
        return m_rotation.getPosition();
    }

    public boolean zeroLimit() {
        return m_extension.limitZero(PIDSparkMax.LimitDirection.kReverse, SparkMaxLimitSwitch.Type.kNormallyClosed);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Arm Extension", () -> getExtension(), (double position) -> setExtension(position));
        builder.addBooleanProperty("Extension Limit Switch", () -> m_extension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed(), null);
        builder.addDoubleProperty("Arm Rotation", () -> getRotation(), null);
    }
}
