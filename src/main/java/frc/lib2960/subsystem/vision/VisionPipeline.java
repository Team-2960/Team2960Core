package frc.lib2960.subsystem.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.config.vision.VisionPipelineConfig;
import frc.lib2960.helper.AngleUtil;

public class VisionPipeline extends SubsystemBase {

    /**
     * Class for storing target results
     */
    public class TargetResults {

        private final Angle camYaw;
        private final Angle camPitch;

        /** Target to Camera transform */
        private final Transform3d targetToCamera;

        /** Target to Robot transform */
        private final Optional<Transform3d> targetToRobot;

        /** PhotonVision target result */
        private final Time timestamp;

        /**
         * Constructor
         * 
         * @param target PhotonVision target result
         */
        public TargetResults(PhotonTrackedTarget target, Time timestamp) {
            this.camYaw = Radians.of(target.getYaw());
            this.camPitch = Radians.of(target.getPitch());

            targetToCamera = new Transform3d(
                    0, 0, 0,
                    new Rotation3d(
                            target.getYaw(),
                            target.getPitch(),
                            0));

            targetToRobot = config.robotToCamera.isPresent()
                    ? Optional.of(targetToCamera.plus(config.robotToCamera.get().inverse()))
                    : Optional.empty();

            this.timestamp = timestamp;
        }

        /**
         * Gets the camera relative orientation of the target
         * 
         * @return camera relative orientation of the target
         */
        public Rotation3d getTargetRotation() {
            return targetToCamera.getRotation();
        }

        /**
         * Gets the robot relative orientation of the target
         * 
         * @return robot relative orientation of the target. Returns camera relative
         *         orientation if robotToCamera is not set.
         */
        public Optional<Rotation3d> getRobotRotation() {
            return targetToRobot.isPresent() ? Optional.of(targetToRobot.get().getRotation())
                    : Optional.empty();
        }

        /**
         * Gets the field relative heading to the target
         * 
         * @param robotPose current robot estimated pose
         * @return Field relative rotation about the z axis. If robotToCamera is not
         *         set, position assumes camera looks down the x-axis of the robot.
         */
        public Angle getFieldHeading(Pose2d robotPose) {
            Transform3d targetToRobot = this.targetToRobot.isPresent() ? this.targetToRobot.get() : targetToCamera;
            Transform3d robotToField = new Transform3d(new Pose3d(robotPose), new Pose3d());
            return targetToRobot.plus(robotToField).getRotation().getMeasureZ();
        }

        /**
         * Gets the elevation angle of the target above the x-y plane.
         * 
         * @return elevation angle of the target above the x-y plane. If robotToCamera
         *         is not set, position assumes camera looks down the x-axis of the
         *         robot.
         */
        public Angle getElevationAngle() {
            return Radians.of(getElevationAngleRad());
        }

        /**
         * Gets the distance of a target from a known height.
         * 
         * @param targetHeight targets height
         * @return distance to the target. If robotToCamera is not set, Optional.empty()
         *         is returned.
         */
        public Optional<Distance> getTargetDist(Distance targetHeight) {
            Optional<Distance> result = Optional.empty();

            if (config.robotToCamera.isPresent()) {
                double height = targetHeight.in(Meters) - targetToRobot.get().getMeasureZ().in(Meters);
                double angle = getElevationAngleRad();

                result = Optional.of(Meters.of(height / Math.tan(angle)));
            }

            return result;
        }

        /**
         * Get the position of the target on the field based on the robot's position and
         * the target's height
         * 
         * @param targetHeight targets height
         * @param robotPose    current robot estimated pose
         * @return position of the target on the field. If robotToCamera is not set,
         *         Optional.empty() is returned.
         */
        public Optional<Translation2d> getTargetPos(Distance targetHeight, Pose2d robotPose) {
            Optional<Translation2d> result = Optional.empty();
            Optional<Distance> dist = getTargetDist(targetHeight);

            if (dist.isPresent()) {
                // Calculate target's position relative to the robot's position
                Translation2d robotToTarget = new Translation2d(
                        dist.get().in(Meters),
                        AngleUtil.toR2d(getFieldHeading(robotPose)));

                // Calculate the target's position on the field
                result = Optional.of(robotPose.getTranslation().plus(robotToTarget));
            }

            return result;
        }

        /**
         * Gets the timestamp the target was found
         * 
         * @return timestamp the target was found
         */
        public Time getTimestamp() {
            return timestamp;
        }

        /**
         * Gets the time since the target was found
         * 
         * @return time since the target was found
         */
        public Time getAge() {
            return Seconds.of(Timer.getFPGATimestamp()).minus(timestamp);
        }

        /**
         * Gets the elevation angle of the target above the x-y plane in radians.
         * 
         * @return elevation angle of the target above the x-y plane in radians. If
         *         robotToCamera is not set, position assumes camera looks down the
         *         x-axis of the robot.
         */
        private double getElevationAngleRad() {
            Transform3d targetToRobot = this.targetToRobot.isPresent() ? this.targetToRobot.get() : targetToCamera;
            var axis = targetToRobot.getRotation().getAxis();
            double xyLen = Math.sqrt(Math.pow(axis.get(0), 2) + Math.pow(axis.get(1), 2));

            return Math.atan2(axis.get(2), xyLen);
        }
    }

    /** Vision pipeline configuration */
    private final VisionPipelineConfig config;

    /** PhotonVision camera reference */
    private final PhotonCamera camera;

    /** Target information for the last target found. If no target has been found */
    Optional<TargetResults> lastTargetResults = Optional.empty();

    /**
     * Target information for the most recent update cycle. Empty if no target was
     * found the last cycle
     */
    Optional<TargetResults> targetResults = Optional.empty();

    /**
     * Constructor
     * 
     * @param config Vision pipeline configuration
     */
    public VisionPipeline(VisionPipelineConfig config) {
        this.config = config;
        this.camera = new PhotonCamera(config.cameraName);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Target Present", targetResults::isPresent, null);
        builder.addStringProperty("Target Yaw",
                () -> targetResults.isPresent() ? targetResults.get().camYaw.toShortString() : "N/A", null);
        builder.addStringProperty("Target Pitch",
                () -> targetResults.isPresent() ? targetResults.get().camPitch.toShortString() : "N/A", null);
        builder.addStringProperty("Last Target Yaw",
                () -> lastTargetResults.isPresent() ? lastTargetResults.get().camYaw.toShortString() : "N/A", null);
        builder.addStringProperty("Last Target Pitch",
                () -> lastTargetResults.isPresent() ? lastTargetResults.get().camPitch.toShortString() : "N/A", null);
        
        // TODO Return robot relative results
    }

    /**
     * Updates camera tracking results
     */
    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<PhotonPipelineResult> latestResult = Optional.empty();
        double maxResultTime = 0;

        // Find latest result
        for (var result : results) {
            if (result.hasTargets()) {
                if (latestResult.isEmpty() || result.getTimestampSeconds() > maxResultTime) {
                    latestResult = Optional.of(result);
                    maxResultTime = result.getTimestampSeconds();
                }
            }
        }

        // Get best target from result if present
        if (latestResult.isPresent() && latestResult.get().hasTargets()) {
            targetResults = Optional.of(
                    new TargetResults(
                            latestResult.get().getBestTarget(),
                            Seconds.of(maxResultTime)));
            lastTargetResults = targetResults;
        } else {
            targetResults = Optional.empty();
        }
    }

    /**
     * Checks if there is a vision target in the camera view
     * 
     * @return true if a vision target was present in the most recent tracking
     *         cycle, false otherwise.
     */
    public boolean isTargetPresent() {
        return targetResults.isPresent();
    }

    /**
     * Gets the latest target result. If no target found in the previous cycle
     * 
     * @return
     */
    public Optional<TargetResults> getTargetResults() {
        return this.targetResults;
    }

    /**
     * Gets the latest target result. If no target found in the previous cycle
     * 
     * @return
     */
    public Optional<TargetResults> getLastTargetResults() {
        return this.lastTargetResults;
    }

}
