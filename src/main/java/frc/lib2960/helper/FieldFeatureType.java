package frc.lib2960.helper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldFeatureType {

    /** Feature type name */
    public final String name;

    /** Map of features by name */
    private final Map<String, FieldFeature> features = new HashMap<>();

    /** Map of features by blue alliance poses */
    private final Map<Pose2d, FieldFeature> blueMap = new HashMap<>();
    
    /** Map of features by red alliance poses */
    private final Map<Pose2d, FieldFeature> redMap = new HashMap<>();

    /** List of blue alliance poses */
    private final List<Pose2d> blueList = new ArrayList<>();

    /** List of red alliance poses */
    private final List<Pose2d> redList = new ArrayList<>();

    /**
     * Constructor
     * 
     * @param name name of the feature type
     */
    public FieldFeatureType(String name) {
        this.name = name;
    }


    /**
     * Adds a field features
     * @param features features to add
     * @return  reference to the current FieldFeatureType for command chaining
     */
    public FieldFeatureType addFeatures(FieldFeature... features) {
        for (var feature : features){
            this.features.put(feature.name, feature);
            blueMap.put(feature.blue, feature);
            redMap.put(feature.red, feature);
            blueList.add(feature.blue);
            redList.add(feature.red);
        }
        return this;
    }

    /**
     * Gets a feature for the type
     * @param name  name of the feature
     * @return  Requested feature if it exists. Optional.empty() otherwise.
     */
    public Optional<FieldFeature> get(String name) {
        return features.containsKey(name) ? Optional.of(features.get(name)) : Optional.empty();
    }

    /**
     * Finds the nearest feature to a given pose
     * @param pose  pose to check
     * @return  nearest 
     */
    public FieldFeature getNearest(Pose2d pose) {
        Map<Pose2d, FieldFeature> map = FieldUtil.isRedAlliance() ? redMap : blueMap;
        List<Pose2d> list = FieldUtil.isRedAlliance() ? redList : blueList;

        // Throw an exception if no features exist
        if(list.size() < 1) throw new RuntimeException("No features found");
        
        return map.get(pose.nearest(list));
    } 
}
