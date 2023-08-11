package frc.robot.subsystems;

public class Arm {
    private static Arm m_instance;

    public static Arm getInstance() {
        if (m_instance == null) {
            m_instance = new Arm();
        }

        return m_instance;
    }

    
}
