package wildlib.utils.ds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleInput {
    private final String m_key;
    private final double m_default;

    public DoubleInput(String key, Double defaultValue) {
        this.m_key = key;
        this.m_default = defaultValue;

        SmartDashboard.putNumber(m_key, defaultValue);
    }

    public DoubleInput(String key) {
        this(key, 0.0);
    }

    public double get(double defaultValue) {
        return SmartDashboard.getNumber(m_key, defaultValue);
    }

    public double get() {
        return SmartDashboard.getNumber(m_key, m_default);
    }

    public void set(double value) {
        SmartDashboard.putNumber(m_key, value);
    }
}
