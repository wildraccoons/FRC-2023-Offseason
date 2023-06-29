package wildlib;
import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    DigitalInput input;
    Normal normal;

    /** If the input is attached to the normally open pin or the normally closed pin of the limit switch. */
    public enum Normal {
        NormallyOpen,
        NormallyClosed,
    }

    LimitSwitch(int channel, Normal normally) {
        input = new DigitalInput(channel);
        normal = normally;
    }

    /** Checks if the limit switch is pressed. */
    public boolean getPressed() {
        switch (normal) {
        case NormallyClosed:
            return input.get();
        case NormallyOpen:
            return !input.get();
        default:
            return true;
        }
    }
}