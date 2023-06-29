package wildlib;
import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    DigitalInput input;
    Polarity polarity;

    /** If the input is attached to the normally open pin or the normally closed pin of the limit switch. */
    public enum Polarity {
        /** 
         * Connected by default.
         * Normally grounded, pulled high by the DIO input when pressed. 
         */
        NormallyOpen,
        /** 
         * Disconnected by default.
         * Normally pulled high by the DIO input, grounded when pressed.
         */
        NormallyClosed,
    }

    /** 
     * Creates a limit switch given a channel.
     * {@code Polarity} defaults to {@link Polarity#NormallyOpen NormallyOpen}.
     * @param channel The DIO channel for the digital input. 0-9 are on-board, 10-25 are on the MXP
     */
    public LimitSwitch(int channel) {
        this(channel, Polarity.NormallyOpen);
    }

    /** 
     * Creates a limit switch given a channel and polarity.
     * @param channel The DIO channel for the digital input. 0-9 are on-board, 10-25 are on the MXP.
     * @param polarity The {@link Polarity polarity} of the limit switch.
     */
    public LimitSwitch(int channel, Polarity polarity) {
        this.input = new DigitalInput(channel);
        this.polarity = polarity;
    }

    /** 
     * Checks if the limit switch is pressed.
     * @return The state of the limit switch.
     */
    public boolean getPressed() {
        switch (polarity) {
        case NormallyClosed:
            return input.get();
        case NormallyOpen:
            return !input.get();
        default:
            return true;
        }
    }
}