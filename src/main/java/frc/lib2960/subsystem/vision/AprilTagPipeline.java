package frc.lib2960.subsystem.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.config.vision.AprilTagPipelineConfig;
import frc.lib2960.subsystem.drivetrain.Drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages connection to a single PhotonVision AprilTag Pipeline
 */
public class AprilTagPipeline extends SubsystemBase {

    /** < Pipeline Settings */
    private final AprilTagPipelineConfig settings;

    /** < Drivetrain object to update */
    private final Drivetrain drivetrain;

    /** < Camera object */
    private final PhotonCamera camera;

    /** < Pose Estimator */
    private final PhotonPoseEstimator pose_est;

    /** < Timestamp of the most recent pose estimation */
    private final MutTime last_timestamp = Seconds.mutable(0);

    /** < Mutable distance for calculating target distance */
    private final MutDistance tagDist = Meters.mutable(0);

    // Shuffleboard
    @SuppressWarnings("unused")
    private final GenericEntry sb_Pose;
    @SuppressWarnings("unused")
    private final GenericEntry sb_lastTimestamp;
    private final GenericEntry sb_aprilTagSeen;

    // Advantage Scope
    private final StructArrayPublisher<Pose3d> as_aprilTags;
    private final StructPublisher<Pose3d> as_cameraPose; // Camera Pose Relative to Robot on the Field
    private final StructPublisher<Pose2d> as_estimatedCameraPose;
    private final ArrayList<Pose3d> aprilTagList = new ArrayList<>();
    private AprilTagFields field;
    private Pose2d last_pose = new Pose2d(); // Do NOT Use for any estimates

    /**
     * Constructor
     * 
     * @param drivetrain Drivetrain object to update with vision measurements
     * @param settings   AprilTagPipeline settings
     * @param cameraName name of the camera
     * @param name
     */
    public AprilTagPipeline(AprilTagPipelineConfig settings, Drivetrain drivetrain, String cameraName) {
        // Initialize settings
        this.settings = settings;

        // Initialize drivetrain
        this.drivetrain = drivetrain;

        // Create Camera
        camera = new PhotonCamera(cameraName);

        // Create pose estimator
        pose_est = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(settings.fieldLayout),
                settings.poseStrategy,
                settings.robotToCamera);

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout(cameraName, BuiltInLayouts.kList)
                .withSize(1, 4);

        sb_Pose = layout.add("Pose" + cameraName, last_pose).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp" + cameraName, last_timestamp).getEntry();
        sb_aprilTagSeen = layout.add(cameraName + "April Tag Read", false).getEntry();

        // Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault()
                .getStructArrayTopic(cameraName, Pose3d.struct).publish();

        as_cameraPose = NetworkTableInstance.getDefault()
                .getStructTopic(cameraName + " pose", Pose3d.struct).publish();

        as_estimatedCameraPose = NetworkTableInstance.getDefault()
                .getStructTopic(cameraName + " Estimated Pose", Pose2d.struct).publish();
    }

    /**
     * Period method. Updates UI.
     */
    @Override
    public void periodic() {
        updatePose();
        updateUI();
    }

    /**
     * Updates camera pose estimation
     */
    private void updatePose() {
        // Create empty list of tracked targets
        List<PhotonTrackedTarget> visionEst = List.of();

        // Get list of all unread vision results
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

        // Clear list of found targets
        aprilTagList.clear();

        // Check every unread result
        for (var result : unreadResults) {
            // Get list of targets found
            visionEst = result.getTargets();

            last_pose = new Pose2d();
            // Check if a pose was estimated
            if (!visionEst.isEmpty()) {
                // Update last timestamp
                last_timestamp.mut_replace(result.getTimestampSeconds(), Seconds);

                // Get found tag count and average distance
                for (var tag : visionEst) {
                    // Get estimated pose
                    var tag_pose3d = PhotonUtils.estimateFieldToRobotAprilTag(
                            tag.bestCameraToTarget,
                            pose_est.getFieldTags().getTagPose(tag.fiducialId).get(),
                            settings.robotToCamera.inverse());

                    // Get ambiguity
                    double ambiguity = tag.getPoseAmbiguity();

                    // Get april tag distance from robot
                    tagDist.mut_replace(
                            tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d()),
                            Meters);

                    // Check limits
                    if (ambiguity <= settings.ambiguityThreshold && tagDist.lte(settings.maxDist)) {
                        // Calculate the current standard deviations
                        Vector<N3> est_std = settings.singleTagSTD;
                        est_std = est_std.times(1 + (tagDist.in(Meters) * tagDist.in(Meters) / 30));

                        // Update last pose
                        last_pose = tag_pose3d.toPose2d();

                        // Add vision measurement to drivetrain
                        drivetrain.addVisionMeasurement(last_pose, last_timestamp, est_std);

                        // Add Found april tag to list of found april tags
                        aprilTagList.add(
                                AprilTagFieldLayout.loadField(field)
                                        .getTagPose(tag.getFiducialId()).get());
                    }
                }
            }
        }
    }

    /**
     * Calculates the camera position
     * @return  camera pose
     */
    public Pose3d getRobotRelativeCamPos() {
        return new Pose3d(drivetrain.getPoseEst()).transformBy(settings.robotToCamera);
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_aprilTagSeen.setBoolean(aprilTagList.size() > 0);

        // Advantage Scope
        as_aprilTags.set((Pose3d[]) aprilTagList.toArray());
        as_cameraPose.set(getRobotRelativeCamPos());
        as_estimatedCameraPose.set(last_pose);
    }
}