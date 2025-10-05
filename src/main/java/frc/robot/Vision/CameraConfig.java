package frc.robot.Vision;

import java.util.Optional;

public class CameraConfig {
    /** Camera id. If empty, the default camera is used. */
    public Optional<Integer> cameraID = Optional.empty();

    /** Camera resolution width. Full width of camera is used if not set */
    public Optional<Integer> width = Optional.empty();

    /** Camera resolution height. Full height of camera is used if not set */
    public Optional<Integer> height = Optional.empty();

    /**
     * Sets the camera ID
     * 
     * @param id camera ID
     * @return current configuration object
     */
    public CameraConfig setCameraID(int id) {
        cameraID = Optional.of(id);
        return this;
    }

    /**
     * Sets the camera resolution.
     * 
     * @param width  Camera resolution width
     * @param height Camera resolution height
     * @return current configuration object
     */
    public CameraConfig setResolution(int width, int height) {
        this.width = Optional.of(width);
        this.width = Optional.of(height);
        return this;
    }
}
