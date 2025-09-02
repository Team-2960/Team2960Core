package frc.lib2960.config.basic;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class RobotCommonConfig {
    private Time updatePeriod = Seconds.of(.020);

    private Mass robotMass = Pounds.of(120);
    private Mass bumperMass = Pounds.of(15);
    private Mass batteryMass = Pounds.of(12.89);

    private Mass totalMass = Pounds.of(robotMass.in(Pounds) + bumperMass.in(Pounds) + batteryMass.in(Pounds));

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
     * Gets the configured update period
     * @return  configured update period
     */
    public Time getUpdatePeriod() {
        return updatePeriod;
    }

    /**
     * Gets the robot's mass without bumpers or battery.
     * @return  robot's mass without bumpers or battery.
     */
    public Mass getRobotMass() {
        return robotMass;
    }


    /**
     * Gets the bumper mass
     * @return  bumper mass
     */
    public Mass getBumperMass() {
        return bumperMass;
    }


    /**
     * Gets the battery mass
     * @return  battery mass
     */
    public Mass getBatteryMass() {
        return batteryMass;
    }


    /**
     * Gets the robot's mass with bumpers or battery.
     * @return  robot's mass with bumpers or battery.
     */
    public Mass getTotalMass() {
        return totalMass;
    }   

    /**
     * Updates the total robot mass
     */
    private void updateTotalMass() {
        totalMass = Pounds.of(robotMass.in(Pounds) + bumperMass.in(Pounds) + batteryMass.in(Pounds));
    }
}
