package frc.lib2960.config.basic;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class RobotCommonConfig {
    /** Update period for the robot. Defaults to 20ms */
    public Time updatePeriod = Seconds.of(.020);

    /** Robot's mass without battery or bumpers. Defaults to 120 lbs. */
    public Mass robotMass = Pounds.of(120);
    /** Robot bumper's mass. Defaults to 15 lbs. */
    public Mass bumperMass = Pounds.of(15);
    /** Robot battery's mass. Defaults to 12.89 lbs. */
    public Mass batteryMass = Pounds.of(12.89);

    /** Robot's mass with bumpers and battery. */
    public Mass totalMass = Pounds.of(robotMass.in(Pounds) + bumperMass.in(Pounds) + batteryMass.in(Pounds));

    private final Map<String, RobotFeature> features = new HashMap<>();

    /**
     * Sets the configured update period. Default is 20ms.
     * 
     * @param updatePeriod update period
     * @return current config object
     */
    public RobotCommonConfig setUpdatePeriod(Time updatePeriod) {
        this.updatePeriod = updatePeriod;
        return this;
    }

    /**
     * Sets the robot mass. Default is 120 lbs.
     * 
     * @param mass robot mass
     * @return current config object
     */
    public RobotCommonConfig setRobotMass(Mass mass) {
        this.robotMass = mass;
        updateTotalMass();
        return this;
    }

    /**
     * Sets the bumper mass. Default is 15 lbs.
     * 
     * @param mass bumper mass
     * @return current config object
     */
    public RobotCommonConfig setBumperMass(Mass mass) {
        this.bumperMass = mass;
        updateTotalMass();
        return this;
    }

    /**
     * Sets the battery mass. Default is 12.89 lbs.
     * 
     * @param mass battery mass
     * @return current config object
     */
    public RobotCommonConfig setBatteryMass(Mass mass) {
        this.batteryMass = mass;
        updateTotalMass();
        return this;
    }

    /**
     * Adds robot features
     * @param features robot features to add
     * @return current config object
     */
    public RobotCommonConfig addFeatures(RobotFeature... features) {
        for(var feature: features) this.features.put(feature.name, feature);
        return this;
    }

    /**
     * Gets a robot feature
     * 
     * @param name  name of the feature
     * @return  Requested feature if it exists. Optional.empty() otherwise
     */
    public Optional<RobotFeature> getFeature(String name) {
        return features.containsKey(name) ? Optional.of(features.get(name)) : Optional.empty();
    }

    /**
     * Updates the total robot mass
     */
    private void updateTotalMass() {
        totalMass = Pounds.of(robotMass.in(Pounds) + bumperMass.in(Pounds) + batteryMass.in(Pounds));
    }
}
