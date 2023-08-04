package wildlib;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/** Utility class for a Spark Max PID controller */
public class PIDSparkMax extends CANSparkMax {
    private static double kP = 0.2; 
    private static double kI = 1e-4;
    private static double kD = 1; 

    private double kIz = 0; 
    private double kFF = 0; 
    
    private double kMaxOutput = 0.75; 
    private double kMinOutput = -0.75;

    private RelativeEncoder encoder;
    private SparkMaxPIDController controller;

    private boolean limitEnabledForward = false;
    private boolean limitEnabledBackward = false;
    private LimitSwitch limitForward;
    private LimitSwitch limitBackward;

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
        this.encoder = super.getEncoder();
        this.controller = super.getPIDController();

        controller.setP(proportional);
        controller.setI(integral);
        controller.setD(derivative);
        
        controller.setIZone(kIz);
        controller.setFF(kFF);
        controller.setOutputRange(kMinOutput, kMaxOutput);
        controller.setFeedbackDevice(encoder);
    }

    /** 
     * Sets the maximum percentage output for the PID controller<br>
     * <strong>Note:</strong>  Because this function wraps
     * {@link com.revrobotics.SparkMaxPIDController#setOutputRange(double, double) setOutputRange()},
     * setting this below zero stops the motor from rotating forwards.
     */
    public void setMaxOutput(double output) {
        this.kMaxOutput = output;
        this.controller.setOutputRange(this.kMinOutput, this.kMaxOutput);
    }

    /** 
     * Sets the minimum percentage output for the PID controller<br>
     * <strong>Note:</strong>  Because this function wraps
     * {@link com.revrobotics.SparkMaxPIDController#setOutputRange(double, double) setOutputRange()},
     * setting this above zero stops the motor from rotating in reverse.
     */
    public void setMinOutput(double output) {
        this.kMinOutput = output;
        this.controller.setOutputRange(this.kMinOutput, this.kMaxOutput);
    }

    /**
     * Enables or disables limit switch checking for the given direction.
     * Must be called with {@link #addLimitSwitch(LimitSwitch, SoftLimitDirection) addLimitSwitch()}
     * in order to take effect.
     * 
     * @param enable Enable ({@code true}) or disable ({@code false}) limit switch checking.
     * @param direction The direction to set.
     */
    public void enableLimitSwitch(boolean enable, CANSparkMax.SoftLimitDirection direction) {
        switch (direction) {
        case kForward:
            this.limitEnabledForward = enable;
        case kReverse:
            this.limitEnabledBackward = enable;
        }
    }

    /** 
     * Sets the limit switch for the given direction.
     * The motor will refuse to turn if it is told to
     * drive past the limit. Must be enabled with
     * {@link #enableLimitSwitch(boolean, SoftLimitDirection) enableLimitSwitch()}
     * in order to take effect.<br><br>
     * <strong>Note:</strong> Since {@link #setReference(double, ControlType) setReference()}
     * sets a value in the physical Spark Max, the motor will continue to try and reach the
     * previously set reference if {@link #setReference(double, ControlType) setReference()}
     * is not called continuously.
     */
    public void addLimitSwitch(LimitSwitch limit, CANSparkMax.SoftLimitDirection direction) {
        switch (direction) {
        case kForward:
            this.limitForward = limit;
        case kReverse:
            this.limitBackward = limit;
        }
    }

    /** Overwrites the current position of the motor encoder. */
    public void setPostion(double pos) {
        this.encoder.setPosition(pos);
    }

    @Override
    public void set(double speed) {
        boolean checkForward = this.limitEnabledForward && this.limitForward != null && this.limitBackward.getPressed();
        boolean checkBackward = this.limitEnabledBackward && this.limitBackward != null && this.limitBackward.getPressed();
        if ((!checkForward || speed < 0) && (!checkBackward || speed > 0)) {
            super.set(speed);
        } else {
            super.set(0);
        }
    }

    /** 
     * Set the controller reference value based on the selected control mode. 
     * 
     * @param value The value to set depending on the control mode. For basic 
     *              duty cycle control this should be a value between -1 and 1.
     * @param ctrl The control type.
     */
    public void setReference(double value, ControlType ctrl) {
        boolean checkForward = this.limitEnabledForward && this.limitForward != null && this.limitBackward.getPressed();
        boolean checkBackward = this.limitEnabledBackward && this.limitBackward != null && this.limitBackward.getPressed();
        switch (ctrl) {
        case kPosition:
            if ((checkForward && value >= this.encoder.getPosition()) || (checkBackward && value <= this.encoder.getPosition())) {
                this.controller.setReference(0, ControlType.kVelocity);
            } else {
                this.stopMotor();
            }
        case kVelocity:
            if ((!checkForward || value < 0) && (!checkBackward || value > 0)) {
                this.controller.setReference(value, ctrl);
            } else {
                this.stopMotor();
            }
        default:
            this.controller.setReference(value, ctrl);
        }
    }

    /** Sets the encoder position to 0 if the specified limit switch is pressed. */
    public boolean limitZero(SoftLimitDirection direction) {
        switch (direction) {
        case kForward:
            if (this.limitEnabledForward && this.limitForward != null && this.limitForward.getPressed()) {
                this.encoder.setPosition(0);
                return true;
            }
        case kReverse:
            if (this.limitEnabledBackward && this.limitBackward != null && this.limitBackward.getPressed()) {
                this.encoder.setPosition(0);
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
        return this.encoder.getPosition();
    }

    /**
     * Overrides the current position of the encoder.
     * 
     * @param position The position to override with.
     */
    public void setEncoderPosition(double position) {
        this.encoder.setPosition(position);
    }

    /** 
     * Returns the state of the {@link LimitSwitch limit switch} added for the
     * given direction. Returns {@code null} if no {@link LimitSwitch limit switch}
     * has been added.
     */
    public Optional<Boolean> getLimit(SoftLimitDirection direction) {
        LimitSwitch read = direction == SoftLimitDirection.kForward ? this.limitForward : this.limitBackward;
        if (read == null) {
            return null;
        } else {
            return Optional.of(read.getPressed());
        }
    }

    /** Get the velocity of the motor from the encoder.
     * This returns the native units of 'RPM' by default.
     * 
     * @return Velocity of the motor in 'RPM'
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }
}