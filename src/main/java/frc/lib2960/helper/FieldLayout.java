package frc.lib2960.helper;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldLayout {
    public final Distance fieldLength;
    public final Distance fieldWidth;

    public final Translation2d fieldCenter;

    private final Map<String, FieldFeatureType> featureTypes = new HashMap<>();

    /**
     * Creates a new layout for the field
     * @param fieldLength   overall length of the playing field
     * @param fieldWidth    overall width of the playing field
     */
    public FieldLayout(Distance fieldLength, Distance fieldWidth) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;

        fieldCenter = new Translation2d(fieldLength.div(2.0), fieldWidth.div(2.0));
    }

    /**
     * Creates new feature types with no features
     * 
     * @param typeNames name of the type
     * @return  reference to the current FieldLayout for command chaining
     */
    public FieldLayout createFeatureTypes(String... typeNames) {
        for(String typeName: typeNames) featureTypes.put(typeName, new FieldFeatureType(typeName));
        return this;
    }


    /**
     * Adds feature types
     * 
     * @param type feature type
     * @return  reference to the current FieldLayout for command chaining
     */
    public FieldLayout addFeatureTypes(FieldFeatureType... types) {
        for (FieldFeatureType type : types)
            featureTypes.put(type.name, type);
        return this;
    }

    /**
     * Adds features to a feature type. If the feature type does not exist, it will be created.
     * @param typeName  name of the feature type
     * @param features  features to add
     * @return  reference to the current FieldLayout for command chaining
     */
    public FieldLayout addFeatures(String typeName, FieldFeature... features) {
        if(!featureTypes.containsKey(typeName)) createFeatureTypes(typeName);

        featureTypes.get(typeName).addFeatures(features);

        return this;
    }

    /**
     * Gets a feature type
     * @param typeName  name of the type
     * @return  Requested feature type if it exists. Optional.empty() otherwise.
     */
    public Optional<FieldFeatureType> getFeatureType(String typeName) {
        return featureTypes.containsKey(typeName) ? Optional.of(featureTypes.get(typeName)) : Optional.empty();
    }


    /**
     * Gets a feature from a given type
     * @param typeName  name of the feature type
     * @param name  name of the feature
     * @return  Requested feature if it exists in the given feature type. Optional.empty() otherwise.
     */
    public Optional<FieldFeature> getFeature(String typeName, String name) {
        Optional<FieldFeature> result = Optional.empty();
        var type = getFeatureType(typeName);
        
        if(type.isPresent()) result = type.get().get(name);

        return result;
    }
}
