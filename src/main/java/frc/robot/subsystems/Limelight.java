package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static Limelight m_instance;
    public static Limelight getInstance() {
        if (m_instance == null) {
            m_instance = new Limelight();
        }

        return m_instance;
    }

    protected NetworkTable m_table;

    protected Limelight() {
        NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * @return Whether the limelight has any valid targets.
     */
    public boolean hasTarget() {
        return m_table.getEntry("tv").getBoolean(false);
    }

    /**
     * @return Horizontal offset from Crosshair to Target (-29.8 to 29.8 degrees).
     */
    public double targetHorizontal() {
        return m_table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical offset from Crosshair to Target (-24.85 to 24.85 degrees).
     */
    public double targetVertical() {
        return m_table.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return Target area (0% to 100% of the image).
     */
    public double targetArea() {
        return m_table.getEntry("ta").getDouble(0.0);
    }

    /**
     * @return The pipeline's latency contribution (ms).
     */
    public long pipelineLatency() {
        return m_table.getEntry("tl").getInteger(0);
    }

    /**
     * @return Capture pipeline latency (ms). 
     *         Time between the end of the exposure of the middle 
     *         row of the sensor to the beginning of the tracking pipeline.
     */
    public long captureLatency() {
        return m_table.getEntry("cl").getInteger(0);
    }

    /**
     * @return Total camera latency (ms).
     *         Equivalent to {@link Limelight#pipelineLatency()} + {@link Limelight#captureLatency()}
     */
    public long totalLatency() {
        return captureLatency() + pipelineLatency();
    }

    /**
     * @return Sidelength of the shortest side of the fitted bounding box (pixels).
     */
    public long shortBounding() {
        return m_table.getEntry("tshort").getInteger(0);
    }

    /**
     * @return Sidelength of the longest side of the fitted bounding box (pixels).
     */
    public long longBounding() {
        return m_table.getEntry("tlong").getInteger(0);
    }

    /**
     * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels).
     */
    public long horizontalBounding() {
        return m_table.getEntry("thor").getInteger(0);
    }

    /**
     * @return Vertical sidelength of the rough bounding box (0 - 320 pixels).
     */
    public long verticalBounding() {
        return m_table.getEntry("tvert").getInteger(0);
    }

    /**
     * @return True active pipeline index of the camera (0..9).
     */
    public long pipeline() {
        return m_table.getEntry("getpipe").getInteger(0);
    }

    /**
     * @return Class ID of the primary neural detector result or the neural classifier result.
     */
    public long classId() {
        return m_table.getEntry("tclass").getInteger(0);
    }

    /**
     * @pre Must be set to an Apriltag pipeline.
     * @return The average HSV color underneath the crosshair region as a Number array.
     */
    public Number[] crosshairHSV() {
        return m_table.getEntry("tc").getNumberArray(new Number[] {});
    }

    /**
     * @pre Must be set to an Apriltag pipeline.
     * @return Robot transform in field-space.
     */
    public Pose3d botpose() {
        return intoPose3d(m_table.getEntry("botpose").getDoubleArray(new double[6]));
    }

    /**
     * @pre Must be set to an Apriltag pipeline.
     * @return Robot transform in field-space (blue driverstation WPILib origin).
     */
    public Pose3d botposeBlue() {
        return intoPose3d(m_table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
    }

    /**
     * @pre Must be set to an Apriltag pipeline.
     * @return Robot transform in field-space (red driverstation WPILib origin).
     */
    public Pose3d botposeRed() {
        return intoPose3d(m_table.getEntry("botpose_wpired").getDoubleArray(new double[6]));
    }

    /**
     * @pre something
     * @return ID of the primary in-view Apriltag.
     */
    public long targetId() {
        return m_table.getEntry("tid").getInteger(0);
    }

    private Pose3d intoPose3d(double[] values) {
        return new Pose3d(
            new Translation3d(values[0], values[1], values[2]),
            new Rotation3d(values[3], values[4], values[5])
        );
    }
}
