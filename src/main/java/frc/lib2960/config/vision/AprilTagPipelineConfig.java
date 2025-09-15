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

    /** Name of the camera*/
    public String cameraName;

    /** Name of the tab the camera telemetry will be displayed on */
    public String uiTabName;
    
    /** < AprilTag Field Layout object */
    public AprilTagFields fieldLayout;
    /** < Robot to Camera transformation */
    public Transform3d robotToCamera;
    /** < Pose Estimation Strategy */
    public PoseStrategy poseStrategy;

    /** < Maximum acceptable target distance from camera in meters */
    public Distance maxDist;
    /** < Single tag standard deviation vector */
    public Vector<N3> singleTagSTD;
    /** < Multi tag standard deviation vector */
    public Vector<N3> multiTagSTD;
    public double ambiguityThreshold;

    /**
     * Constructor
     * 
     * @param fieldLayout        AprilTag Field Layout object
     * @param robotToCamera      Robot to Camera transformation
     * @param poseStrategy       Pose Estimation Strategy
     * @param maxDist            Maximum acceptable target distance from camera in
     *                           meters
     * @param singleTagSTD       Single tag standard deviation vector
     * @param multiTagSTD        Multi tag standard deviation vector
     * @param ambiguityThreshold The max april tag ambiguity the camera will accept
     */
    public AprilTagPipelineConfig(
            AprilTagFields fieldLayout,
            Transform3d robotToCamera,
            PoseStrategy poseStrategy,
            Distance maxDist,
            Vector<N3> singleTagSTD,
            Vector<N3> multiTagSTD,
            double ambiguityThreshold) {
        this.fieldLayout = fieldLayout;
        this.robotToCamera = robotToCamera;
        this.poseStrategy = poseStrategy;
        this.maxDist = maxDist;
        this.singleTagSTD = singleTagSTD;
        this.multiTagSTD = multiTagSTD;
        this.ambiguityThreshold = ambiguityThreshold;
    }

    /**
     * Constructor.
     * - name is set to camera_name
     * - fieldlayout is set to AprilTagFields.kDefault (current season)
     * - poseStrategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSO
     * 
     * @param camera_name        Camera name for the pipeline as configured in
     *                           PhotoVision
     * @param robottocamera      Robot to Camera transformation
     * @param maxDist            Maximum acceptable target distance from camera in
     *                           meters
     * @param singleTagSTD       Single tag standard deviation vector
     * @param multiTagSTD        Multi tag standard deviation vector
     * @param ambiguityThreshold The max april tag ambiguity the camera will accept
     */
    public AprilTagPipelineConfig(
            Transform3d robotToCamera,
            Distance maxDist,
            Vector<N3> singleTagSTD,
            Vector<N3> multiTagSTD,
            double ambiguityThreshold) {

        this(
                AprilTagFields.kDefaultField,
                robotToCamera,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                maxDist,
                singleTagSTD,
                multiTagSTD,
                ambiguityThreshold);
    }

    /**
     * Constructor.
     * - name is set to camera_name
     * - fieldlayout is set to AprilTagFields.kDefault (current season)
     * - poseStrategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
     * - maxDist is set to 4 meters
     * - singleTagSTD is set to VecBuilder.fill(4, 4, 8)
     * - multiTagSTD is set to VecBuilder.fill(0.5, 0.5, 1)
     * - ambiguityThreshold is set to 0
     * 
     * @param camera_name   Camera name for the pipeline as configured in
     *                      PhotoVision
     * @param robotToCamera Robot to Camera transformation
     */
    public AprilTagPipelineConfig(Transform3d robotToCamera) {

        this(
                AprilTagFields.kDefaultField,
                robotToCamera,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Meters.of(4),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1),
                0);
    }
}