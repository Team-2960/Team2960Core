package frc.lib2960.config.vision;

import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

/**
 * Apriltag Pipeline Settings
 */
public class AprilTagPipelineConfig {

    /** Name of the camera */
    public String cameraName;

    /**
     * Name of the tab the camera telemetry will be displayed on. Defaults to
     * "Vision"
     */
    public String uiTabName = "Vision";

    /** < Robot to Camera transformation */
    public Transform3d robotToCamera;

    /** < AprilTag Field Layout object */
    public AprilTagFields fieldLayout = AprilTagFields.kDefaultField;

    /** < Pose Estimation Strategy */
    public PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /** < Single tag standard deviation vector */
    public Vector<N3> singleTagSTD = VecBuilder.fill(4, 4, 8);

    /** < Multi tag standard deviation vector */
    public Vector<N3> multiTagSTD = VecBuilder.fill(0.5, 0.5, 1);

    /** < Maximum acceptable target distance from camera in meters */
    public Distance maxDist = Meters.of(4);

    /** < Maximum acceptable ambiguity value */
    public double ambiguityThreshold = 0;

    /**
     * Constructor.
     * 
     * @param camera_name   Camera name for the pipeline as configured in
     *                      PhotoVision
     * @param robotToCamera Robot to Camera transformation
     */
    public AprilTagPipelineConfig(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
    }

    /**
     * Set the UI Tab Name. Defaults to "Vision"
     * @param uiTabName UI Tab Name
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setUITabName(String uiTabName) {
        this.uiTabName = uiTabName;
        return this;
    }

    /**
     * Set the AprilTag Field Layout. Defaults to AprilTagFields.kDefaultField
     * 
     * @param fieldLayout april tag field layout
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setFieldLayout(AprilTagFields fieldLayout) {
        this.fieldLayout = fieldLayout;
        return this;
    }

    /**
     * Set the Pose estimation strategy. Defaults to
     * PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
     * 
     * @param poseStrategy pose strategy
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setPoseStrategy(PoseStrategy poseStrategy) {
        this.poseStrategy = poseStrategy;
        return this;
    }

    /**
     * Set the seed standard deviations used for a single tag detection. Defaults to
     * [4.0, 4.0, 8.0]
     * 
     * @param singleTagSTD seed standard deviations for a single tag detection
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setSingleTagSTD(Vector<N3> singleTagSTD) {
        this.singleTagSTD = singleTagSTD;
        return this;
    }

    /**
     * Set the seed standard deviations used for a multi tag detection. Defaults to
     * [4.0, 4.0, 8.0]
     * 
     * @param multiTagSTD seed standard deviations for a multi tag detection
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setMultiTagSTD(Vector<N3> multiTagSTD) {
        this.multiTagSTD = multiTagSTD;
        return this;
    }

    /**
     * Set the maximum detection distance for april tags. Defaults to 4 meters
     * 
     * @param maxDist maximum detection distance for April Tags
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setMaxDist(Distance maxDist) {
        this.maxDist = maxDist;
        return this;
    }

    /**
     * Set the maximum acceptable ambiguity value. Defaults to 0.
     * 
     * @param ambiguityThreshold Maximum acceptable ambiguity value
     * @return Current configuration object
     */
    public AprilTagPipelineConfig setAmbiguityThreshold(double ambiguityThreshold) {
        this.ambiguityThreshold = ambiguityThreshold;
        return this;
    }
}