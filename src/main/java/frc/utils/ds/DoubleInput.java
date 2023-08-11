package frc.utils.ds;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleInput {
    private final GenericSubscriber m_subscriber;
    private final String m_key;
    private final double m_default;

    public DoubleInput(String key, Double defaultValue) {
        this.m_key = key;
        this.m_default = defaultValue;

        SmartDashboard.putNumber(m_key, defaultValue);
        this.m_subscriber = SmartDashboard.getEntry(m_key).getTopic().genericSubscribe("Double");
    }

    public DoubleInput(String key) {
        this(key, 0.0);
    }

    public double get(double defaultValue) {
        return m_subscriber.getDouble(defaultValue);
    }

    public double get() {
        return m_subscriber.getDouble(m_default);
    }

    public void set(double value) {
        SmartDashboard.putNumber(m_key, value);
    }
}
