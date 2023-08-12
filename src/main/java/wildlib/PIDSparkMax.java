package wildlib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

/** Utility class for a Spark Max PID controller */
public class PIDSparkMax extends CANSparkMax {
    private static double kP = 0.2; 
    private static double kI = 1e-4;
    private static double kD = 0; 

    private double kIz = 0; 
    private double kFF = 0; 
    
    private double kMaxOutput = 0.75; 
    private double kMinOutput = -0.75;

    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_controller;

    public enum LimitDirection {
        kForward,
        kReverse,
    }

    /** Creates a new {@link PIDSparkMax} with default {@code kP}, {@code kI}, {@code kD} values.
     * <br><br>
     * Default values are:<br><br>
     * <strong><code>kP:</code></strong> <code>0.2</code> <br><br>
     * <strong><code>kI:</code></strong> <code>1e-4</code> <br><br>
     * <strong><code>kD:</code></strong> <code>1</code>
     * 
     * @param deviceId The motor CAN Id
     * @param type The motor type connected to the controller. 
     *             Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. 
     *             Brushed motors must be connected to the Red and Black terminals only.
     */
    public PIDSparkMax(int deviceId, MotorType type) {
        this(deviceId, type, kP, kI, kD);
    }

    /** Creates a new {@link PIDSparkMax} with specified {@code kP}, {@code kI}, {@code kD} values.
     * 
     * @param deviceId The motor CAN Id
     * @param type The motor type connected to the controller. 
     *             Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. 
     *             Brushed motors must be connected to the Red and Black terminals only.
     * @param proportional Sets the {@code kP} of the PID.
     * @param integral Sets the {@code kI} of the PID.
     * @param derivative Sets the {@code kD} of the PID.
     */
    public PIDSparkMax(int deviceId, MotorType type, double proportional, double integral, double derivative) {
        super(deviceId, type);
        m_encoder = super.getEncoder();
        m_controller = super.getPIDController();

        m_controller.setP(proportional);
        m_controller.setI(integral);
        m_controller.setD(derivative);
        
        m_controller.setIZone(kIz);
        m_controller.setFF(kFF);
        m_controller.setOutputRange(kMinOutput, kMaxOutput);
    }

    /** 
     * Sets the maximum percentage output for the PID controller<br>
     * <strong>Note:</strong>  Because this function wraps
     * {@link com.revrobotics.SparkMaxPIDController#setOutputRange(double, double) setOutputRange()},
     * setting this below zero stops the motor from rotating forwards.
     */
    public void setMaxOutput(double output) {
        kMaxOutput = output;
        m_controller.setOutputRange(this.kMinOutput, this.kMaxOutput);
    }

    /** 
     * Sets the minimum percentage output for the PID controller<br>
     * <strong>Note:</strong>  Because this function wraps
     * {@link com.revrobotics.SparkMaxPIDController#setOutputRange(double, double) setOutputRange()},
     * setting this above zero stops the motor from rotating in reverse.
     */
    public void setMinOutput(double output) {
        kMinOutput = output;
        m_controller.setOutputRange(this.kMinOutput, this.kMaxOutput);
    }

    /** 
     * Set the controller reference value based on the selected control mode.
     * 
     * @param value The value to set depending on the control mode. 
     *              For basic duty cycle control this should be a value between -1 and 1 
     *              Otherwise: Voltage Control: Voltage (volts) 
     *              Velocity Control: Velocity (RPM) 
     *              Position Control: Position (Rotations) 
     *              Current Control: Current (Amps). 
     *              Native units can be changed using the setPositionConversionFactor() or 
     *              setVelocityConversionFactor() methods of the CANEncoder class.
     * @param ctrl The control type.
     * @return {@link REVLibError#kOk} if successful.
     */
    public REVLibError setReference(double value, ControlType ctrl) {
        return m_controller.setReference(value, ctrl);
    }

    /** 
     * Overwrites the current position of the motor encoder.
     * 
     * @param position The reference position to use in the PID.
     * @return {@link REVLibError#kOk} if successful.
     */
    public REVLibError setTargetPostion(double position) {
        return m_controller.setReference(position, ControlType.kPosition);
    }

    /**
     * Set the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position.
     * 
     * @param factor The conversion factor to multiply the native unit by.
     * @return {@link REVLibError#kOk} if successful.
     */
    public REVLibError setPositionConversionFactor(double factor) {
        return m_encoder.setPositionConversionFactor(factor);
    }

    /**
     * Set the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity.
     * 
     * @param factor The conversion factor to multiply the native unit by.
     * @return {@link REVLibError#kOk} if successful.
     */
    public REVLibError setVelocityConversionFactor(double factor) {
        return m_encoder.setVelocityConversionFactor(factor);
    }

    /**
     * Zeros the encoder if the specified limit switch is pressed.
     * This call will disable support for the alternate encoder.
     * 
     * @param direction Which limit switch to check.
     * @param polarity The polarity of the specified limit switch.
     * @return {@code true} if the encoder was zeroed.
     */
    public boolean limitZero(LimitDirection direction, SparkMaxLimitSwitch.Type polarity) {
        switch (direction) {
        case kForward:
            SparkMaxLimitSwitch forwardLimit = super.getForwardLimitSwitch(polarity);
            if (forwardLimit.isLimitSwitchEnabled() && forwardLimit.isPressed()) {
                setEncoderPosition(0.0);
                return true;
            }
        case kReverse:
            SparkMaxLimitSwitch reverseLimit = super.getReverseLimitSwitch(polarity);
            if (reverseLimit.isLimitSwitchEnabled() && reverseLimit.isPressed()) {
                setEncoderPosition(0.0);
                return true;
            }
        }

        return false;
    }

    /** 
     * Gets the position recorded by the {@link com.revrobotics.RelativeEncoder Relative Encoder}.
     * 
     * @return Relative position in Rotations
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Overrides the current position of the encoder.
     * 
     * @param position The position to override with.
     */
    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    /** Get the velocity of the motor from the encoder.
     * This returns the native units of 'RPM' by default.
     * 
     * @return Velocity of the motor in 'RPM'
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }
}