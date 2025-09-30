package frc.lib2960.config.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;

public class VisionPipelineConfig {
    /** Name of the camera */
    public String cameraName;

    /**
     * Name of the tab the camera telemetry will be displayed on. Defaults to
     * "Vision"
     */
    public String uiTabName = "Vision";

    /** < Robot to Camera transformation */
    public Optional<Transform3d> robotToCamera = Optional.empty();

    /** < Result Timeout */
    public Time resultTimeout = Seconds.of(0.167);

    /**
     * Constructor
     * 
     * @param cameraName Name of the camera
     */
    public VisionPipelineConfig(String cameraName) {
        this.cameraName = cameraName;
    }

    /**
     * Set the UI Tab Name. Defaults to "Vision"
     * @param uiTabName UI Tab Name
     * @return Current configuration object
     */
    public VisionPipelineConfig setUITabName(String uiTabName) {
        this.uiTabName = uiTabName;
        return this;
    }

    /**
     * Set the robot to camera transform. Defaults to empty.
     * @param robotToCamera robot to camera transform
     * @return Current configuration object
     */
    public VisionPipelineConfig setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = Optional.of(robotToCamera);
        return this;
    }

    /**
     * Clears the robot to camera transform.
     * @return Current configuration object
     */
    public VisionPipelineConfig resetRobotToCamera() {
        this.robotToCamera =  Optional.empty();
        return this;
    }

}
