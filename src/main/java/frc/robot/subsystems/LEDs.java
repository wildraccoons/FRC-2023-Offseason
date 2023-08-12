package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private static LEDs m_instance;
    public static LEDs getInstance() {
        if (m_instance == null) {
            m_instance = new LEDs(LEDConstants.kLength);
        }

        return m_instance;
    }

    private enum PatternType {
        kOff, kSolid, kFlashing, kHeartBeat, kChaserUp, kChaserDown,
    }

    public enum Pattern {
        Off(PatternType.kOff, Double.POSITIVE_INFINITY, Color.kBlack),
        Emergency(PatternType.kFlashing, 0.2, Color.kRed, Color.kBlack);

        /** Type of pattern to display. */
        PatternType type;
        /** Array of colors to iterate over. */
        Color[] colors;
        /** Time in seconds between states. */
        double interval;

        private Pattern(PatternType type, double interval, Color... colors) {
            this.type = type;
            this.colors = colors;
            this.interval = interval;
        }

        private Pattern(Color color) {
            this.type = PatternType.kSolid;
            this.interval = Double.POSITIVE_INFINITY;
            this.colors = new Color[] { color };
        }
    }

    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_buffer;

    private LEDs(int length) {
        m_leds = new AddressableLED(IOConstants.ledPort);

        m_leds.setLength(length);
        m_buffer = new AddressableLEDBuffer(length);

        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void periodic() {
        
    }

    public Color getLED(int index) {
        return m_buffer.getLED(index);
    }

    public Color8Bit getLED8Bit(int index) {
        return m_buffer.getLED8Bit(index);
    }

    public void setLED(int index, Color color) {
        m_buffer.setLED(index, color);
    }

    public void setLED(int index, Color8Bit color) {
        m_buffer.setLED(index, color);
    }

    public void setRGB(int index, byte r, byte g, byte b) {
        m_buffer.setRGB(index, r, g, b);
    }

    public void setHSV(int index, byte h, byte s, byte v) {
        m_buffer.setHSV(index, h, s, v);
    }

    public int getLength() {
        return m_buffer.getLength();
    }

    public void stop() {
        m_leds.stop();
    }
}
