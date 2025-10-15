package frc.lib2960.helper;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldFeature {
    /** Name of the feature */
    public final String name;

    /** Blue alliance feature pose */
    public final Pose2d blue;

    /** Red alliance feature pose */
    public final Pose2d red;

    /** Map of sub features */
    private final Map<String, FieldFeature> subFeatures = new HashMap<>();

    /**
     * Sets the pose of a feature on the robot field
     * 
     * @param name name of the feature
     * @param blue pose for the blue alliance
     * @param red  pose for the red alliance
     */
    public FieldFeature(String name, Pose2d blue, Pose2d red) {
        this.name = name;
        this.blue = blue;
        this.red = red;
    }

    /**
     * Sets the pose of a feature that is common for both alliances
     * 
     * @param name name of the feature
     * @param pose pose of the feature
     */
    public FieldFeature(String name, Pose2d pose) {
        this(name, pose, pose);
    }

    /**
     * Sets the position of a feature on the robot field
     * 
     * @param name name of the feature
     * @param blue position for the blue alliance
     * @param red  position for the red alliance
     */
    public FieldFeature(String name, Translation2d blue, Translation2d red) {
        this.name = name;
        this.blue = new Pose2d(blue, new Rotation2d());
        this.red = new Pose2d(red, new Rotation2d());
        ;
    }

    /**
     * Sets the position of a feature that is common for both alliances
     * 
     * @param name     name of the feature
     * @param position position of the feature
     */
    public FieldFeature(String name, Translation2d position) {
        this(name, position, position);
    }

    /**
     * Sets the rotation of a feature on the robot field
     * 
     * @param name name of the feature
     * @param blue rotation for the blue alliance
     * @param red  rotation for the red alliance
     */
    public FieldFeature(String name, Rotation2d blue, Rotation2d red) {
        this.name = name;
        this.blue = new Pose2d(new Translation2d(), blue);
        this.red = new Pose2d(new Translation2d(), red);
        ;
    }

    /**
     * Sets the rotation of a feature that is common for both alliances
     * 
     * @param name     name of the feature
     * @param rotation rotation of the feature
     */
    public FieldFeature(String name, Rotation2d rotation) {
        this(name, rotation, rotation);
    }

    /**
     * Adds sub features 
     * @param name name of the sub feature
     * @param features sub features to add
     * @return reference to the current feature for method chaining
     */
    public FieldFeature addSubFeature(String name, FieldFeature feature) {
        subFeatures.put(name, feature);
        return this;
    }

    /**
     * Gets a sub feature by name
     * @param name name of the sub feature
     * @return  Requested sub feature if it exists. Optional.empty() otherwise.
     */
    public Optional<FieldFeature> getSubFeature(String name) {
        return subFeatures.containsKey(name) ? Optional.of(subFeatures.get(name)) : Optional.empty();
    }

    /**
     * Gets the feature's pose for the current alliance
     * 
     * @return the feature's pose for the current alliance
     */
    public Pose2d pose() {
        return FieldUtil.isRedAlliance() ? red : blue;
    }

    /**
     * Gets the feature's position for the current alliance
     * 
     * @return the feature's position for the current alliance
     */
    public Translation2d position() {
        return pose().getTranslation();
    }

    /**
     * Gets the feature's rotation for the current alliance
     * 
     * @return the feature's rotation for the current alliance
     */

    public Rotation2d rotation() {
        return pose().getRotation();
    }
}
