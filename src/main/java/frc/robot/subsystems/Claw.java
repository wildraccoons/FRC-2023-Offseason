package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.IOConstants;
import wildlib.PIDSparkMax;
import wildlib.utils.MathUtils;

public class Claw extends SubsystemBase {
    private static Claw m_instance;
    public static Claw getInstance() {
        if (m_instance == null) {
            m_instance = new Claw(
                new PIDSparkMax(
                    IOConstants.clawContractionId,
                    MotorType.kBrushless,
                    ClawConstants.kContractionP,
                    ClawConstants.kContractionI,
                    ClawConstants.kContractionD
                ),
                new PIDSparkMax(
                    IOConstants.clawRotationId,
                    MotorType.kBrushless,
                    ClawConstants.kRotationP,
                    ClawConstants.kRotationI,
                    ClawConstants.kRotationD
                )
            );
        }

        return m_instance;
    }

    private final PIDSparkMax m_contraction;
    private final PIDSparkMax m_rotation;

    private Claw(PIDSparkMax contraction, PIDSparkMax rotation) {
        m_contraction = contraction;
        m_rotation = rotation;

        // Configure rotation
        m_rotation.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_rotation.setSoftLimit(SoftLimitDirection.kReverse, ClawConstants.kMinRotation);

        m_rotation.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_rotation.setSoftLimit(SoftLimitDirection.kForward, ClawConstants.kMaxRotation);

        m_rotation.setPositionConversionFactor(ClawConstants.kRotationConversionFactor);
        m_rotation.setVelocityConversionFactor(ClawConstants.kRotationConversionFactor);

        m_rotation.setMaxOutput(0.2);
        m_rotation.setMinOutput(-0.1);

        // Configure contraction
        m_contraction.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_contraction.setSoftLimit(SoftLimitDirection.kReverse, ClawConstants.kMinContraction);

        m_contraction.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_contraction.setSoftLimit(SoftLimitDirection.kForward, ClawConstants.kMaxContraction);

        m_contraction.setPositionConversionFactor(ClawConstants.kContractionConversionFactor);
        m_contraction.setVelocityConversionFactor(ClawConstants.kContractionConversionFactor);
    }

    @Override
    public void periodic() {
        
    }

    /** 
     * Sets the speed of the rotation motor.
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setRotation(double speed) {
        m_rotation.set(speed);
    }

    public void stopRotation() {
        m_rotation.stopMotor();
    }

    public REVLibError setRotationPosition(double position) {
        return m_rotation.setReference(position, ControlType.kPosition);
    }

    public REVLibError setRotationVelocity(double velocity) {
        return m_rotation.setReference(velocity, ControlType.kVelocity);
    }
    
    public CommandBase setRotationAndWait(double position, double range) {
        System.out.println("claw rotation called");
        return new InstantCommand(() -> setRotationPosition(position))
            .andThen(new WaitUntilCommand(() -> {
                return MathUtils.closeEnough(getRotation(), position, range);
            }));
    }

    public REVLibError holdRotation() {
        return setRotationPosition(getRotation());
    }

    public double getRotation() {
        return m_rotation.getPosition();
    }

    public void setContraction(double speed) {
        m_contraction.set(speed);
    }

    public void stopContraction() {
        m_contraction.stopMotor();
    }

    public REVLibError setContractionPosition(double position) {
        return m_contraction.setReference(position, ControlType.kPosition);
    }

    public REVLibError setContractionVelocity(double velocity) {
        return m_contraction.setReference(velocity, ControlType.kVelocity);
    }

    public CommandBase setContractionAndWait(double position, double range) {
        System.out.println("contraction called");
        return new InstantCommand(() -> setContractionPosition(position))
            .andThen(new WaitUntilCommand(() -> {
                return MathUtils.closeEnough(getContraction(), position, range);
            }));
    }

    public double getContraction() {
        return m_contraction.getPosition();
    }

    public void zeroSensors() {
        m_contraction.setEncoderPosition(0.0);
        m_rotation.setEncoderPosition(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Claw Contraction", () -> m_contraction.getPosition(), (position) -> setContractionPosition(position));
        builder.addDoubleProperty("Claw Rotation", () -> m_rotation.getPosition(), (position) -> setRotationPosition(position));
    }
}
