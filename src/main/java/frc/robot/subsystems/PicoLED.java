package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;

public class PicoLED extends SubsystemBase {
    private static PicoLED m_instance;
    public static PicoLED getInstance() {
        if (m_instance == null) {
            m_instance = new PicoLED();
        }

        return m_instance;
    }

    private final I2C m_device;

    public enum RecvError {
        kOk,
        kMissingData,
        kInvalidPatternType,
        kOutOfMemory,
    }

    public enum PatternType {
        kOff,
        kSolid,
        kFlashing,
        kHeartBeat,
        kChaserUp,
        kChaserDown,
    }

    public enum Pattern {
        Ok;

        PatternType type;
        double interval;
        Color8Bit[] colors;
    }

    private PicoLED() {
        m_device = new I2C(I2C.Port.kOnboard, IOConstants.kLedAddr);
    }

    /**
     * Attempts to access the device on the I2C bus.
     *
     * @return Connection successful: {@code true} for success, {@code false} for aborted.
     */
    public boolean verifyConnection() {
        return !m_device.addressOnly();
    }

    public RecvError sendPattern(Pattern pattern) {
        // Probably a better way of doing this,
        // but I want to check out how I2C.transaction
        // works before I use it.
        m_device.writeBulk(new byte[] {});
        byte[] output = new byte[] { 0x00 };
        m_device.read(0x00, 1, output);
        return RecvError.values()[output[0]];
    }
}
