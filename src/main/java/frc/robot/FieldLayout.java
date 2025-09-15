package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldLayout {
        public static final Transform2d fieldCenterOffset = new Transform2d(
                        Meters.of(8.270875),
                        Meters.of(4.105275), new Rotation2d(0.0));

        public enum StageFace {
                AMP,
                SOURCE,
                FAR
        }

        public enum ReefFace {
                CENTER,
                ZERO,
                SIXTY,
                ONETWENTY,
                ONEEIGHTY,
                TWOFOURTY,
                THREEHUNDRED,
        }

        public enum ReefBranchOffset {
                LEFT, MIDDLE, RIGHT
        }

        public enum BranchID {
                A, B, C, D, E, F, G, H, I, J, K, L
        }

        public enum AlgaeType {
                LEFT_ALGAE,
                MIDDLE_ALGAE,
                RIGHT_ALGAE
        }

        public enum NoteType {
                NEAR_STAGE,
                NEAR_SPEAKER,
                NEAR_AMP,
                FAR_SOURCE2,
                FAR_SOURCE1,
                FAR_MID,
                FAR_AMP1,
                FAR_AMP2
        }

        public static final Pose2d bProcessor = new Pose2d(
                        Meters.of(5.973),
                        Meters.of(0),
                        Rotation2d.fromDegrees(0));

        public static final Pose2d bHPRight = new Pose2d(
                        Meters.of(0.838),
                        Meters.of(0.657),
                        Rotation2d.fromDegrees(54));
        public static final Pose2d bHPLeft = new Pose2d(
                        Meters.of(0.838),
                        Meters.of(7.395),
                        Rotation2d.fromDegrees(-54));

        public static final Pose2d bReefCenter = new Pose2d(
                        Meters.of(4.475),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bReef0 = new Pose2d(
                        Meters.of(3.643),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bReef60 = new Pose2d(
                        Meters.of(4.059),
                        Meters.of(3.306),
                        Rotation2d.fromDegrees(60));
        public static final Pose2d bReef120 = new Pose2d(
                        Meters.of(4.891),
                        Meters.of(3.306),
                        Rotation2d.fromDegrees(120));
        public static final Pose2d bReef180 = new Pose2d(
                        Meters.of(5.662),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(180));
        public static final Pose2d bReef240 = new Pose2d(
                        Meters.of(5.096),
                        Meters.of(5.054),
                        Rotation2d.fromDegrees(-120));
        public static final Pose2d bReef300 = new Pose2d(
                        Meters.of(4.059),
                        Meters.of(4.746),
                        Rotation2d.fromDegrees(-60));

        public static final Pose2d bBranchA = new Pose2d(
                        Meters.of(3.643),
                        Meters.of(4.191),
                        Rotation2d.fromDegrees(0.000));
        public static final Pose2d bBranchB = new Pose2d(
                        Meters.of(3.643),
                        Meters.of(3.861),
                        Rotation2d.fromDegrees(0.000));
        public static final Pose2d bBranchC = new Pose2d(
                        Meters.of(3.917),
                        Meters.of(3.388),
                        Rotation2d.fromDegrees(60.000));
        public static final Pose2d bBranchD = new Pose2d(
                        Meters.of(4.202),
                        Meters.of(3.223),
                        Rotation2d.fromDegrees(60.000));
        public static final Pose2d bBranchE = new Pose2d(
                        Meters.of(4.748),
                        Meters.of(3.223),
                        Rotation2d.fromDegrees(120.000));
        public static final Pose2d bBranchF = new Pose2d(
                        Meters.of(5.033),
                        Meters.of(3.388),
                        Rotation2d.fromDegrees(120.000));
        public static final Pose2d bBranchG = new Pose2d(
                        Meters.of(5.307),
                        Meters.of(3.861),
                        Rotation2d.fromDegrees(180.000));
        public static final Pose2d bBranchH = new Pose2d(
                        Meters.of(5.307),
                        Meters.of(4.191),
                        Rotation2d.fromDegrees(180.000));
        public static final Pose2d bBranchI = new Pose2d(
                        Meters.of(5.033),
                        Meters.of(4.664),
                        Rotation2d.fromDegrees(240.000));
        public static final Pose2d bBranchJ = new Pose2d(
                        Meters.of(4.748),
                        Meters.of(4.828),
                        Rotation2d.fromDegrees(240.000));
        public static final Pose2d bBranchK = new Pose2d(
                        Meters.of(4.202),
                        Meters.of(4.828),
                        Rotation2d.fromDegrees(300.000));
        public static final Pose2d bBranchL = new Pose2d(
                        Meters.of(4.748),
                        Meters.of(4.664),
                        Rotation2d.fromDegrees(300.000));

        public static final Pose2d bAlgaeLeft = new Pose2d(
                        Meters.of(1.205),
                        Meters.of(2.197),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bAlgaeMiddle = new Pose2d(
                        Meters.of(1.205),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bAlgaeRight = new Pose2d(
                        Meters.of(0.838),
                        Meters.of(5.855),
                        Rotation2d.fromDegrees(0));

        public static final Pose2d bCageProcessor = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(0.792),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bCageMiddle = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(1.883),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d bCageTable = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(2.973),
                        Rotation2d.fromDegrees(0));

        public static final List<Pose2d> bReefFaces = Arrays.asList(
                        bReefCenter,
                        bReef0,
                        bReef60,
                        bReef120,
                        bReef180,
                        bReef240,
                        bReef300);

        public static final Map<ReefFace, Pose2d> bReef = Map.of(
                        ReefFace.CENTER, bReefCenter,
                        ReefFace.ZERO, bReef0,
                        ReefFace.SIXTY, bReef60,
                        ReefFace.ONETWENTY, bReef120,
                        ReefFace.ONEEIGHTY, bReef180,
                        ReefFace.TWOFOURTY, bReef240,
                        ReefFace.THREEHUNDRED, bReef300);

        public static final List<Pose2d> bBranchList = Arrays.asList(
                        bBranchA,
                        bBranchB,
                        bBranchC,
                        bBranchD,
                        bBranchE,
                        bBranchF,
                        bBranchG,
                        bBranchH,
                        bBranchI,
                        bBranchJ,
                        bBranchK,
                        bBranchL);

        public static final Map<Pose2d, Map<ReefBranchOffset, Pose2d>> bReefPoseSets = Map.of(
                        bReef0, Map.of(
                                        ReefBranchOffset.LEFT, bBranchA,
                                        ReefBranchOffset.MIDDLE, bReef0,
                                        ReefBranchOffset.RIGHT, bBranchB),
                        bReef60, Map.of(
                                        ReefBranchOffset.LEFT, bBranchC,
                                        ReefBranchOffset.MIDDLE, bReef60,
                                        ReefBranchOffset.RIGHT, bBranchD),
                        bReef120, Map.of(
                                        ReefBranchOffset.RIGHT, bBranchE,
                                        ReefBranchOffset.MIDDLE, bReef120,
                                        ReefBranchOffset.LEFT, bBranchF),
                        bReef180, Map.of(
                                        ReefBranchOffset.RIGHT, bBranchG,
                                        ReefBranchOffset.MIDDLE, bReef180,
                                        ReefBranchOffset.LEFT, bBranchH),
                        bReef240, Map.of(
                                        ReefBranchOffset.RIGHT, bBranchI,
                                        ReefBranchOffset.MIDDLE, bReef240,
                                        ReefBranchOffset.LEFT, bBranchJ),
                        bReef300, Map.of(
                                        ReefBranchOffset.LEFT, bBranchK,
                                        ReefBranchOffset.MIDDLE, bReef300,
                                        ReefBranchOffset.RIGHT, bBranchL));

        public static final Map<AlgaeType, Pose2d> bAlgaeType = Map.of(
                        AlgaeType.LEFT_ALGAE, bAlgaeLeft,
                        AlgaeType.MIDDLE_ALGAE, bAlgaeMiddle,
                        AlgaeType.RIGHT_ALGAE, bAlgaeRight);

        public static final Distance bAutoLineX = Meters.of(-6.137 + fieldCenterOffset.getX());

        public static final HashMap<BranchID, Pose2d> bBranchMap = new HashMap<>();

        public static final Pose2d rProcessor = new Pose2d(
                        Meters.of(11.54),
                        Meters.of(8.051),
                        Rotation2d.fromDegrees(0));

        public static final Pose2d rHPRight = new Pose2d(
                        Meters.of(16.681),
                        Meters.of(7.395),
                        Rotation2d.fromDegrees(54));
        public static final Pose2d rHPLeft = new Pose2d(
                        Meters.of(16.681),
                        Meters.of(0.657),
                        Rotation2d.fromDegrees(-54));

        public static final Pose2d rReefCenter = new Pose2d(
                        Meters.of(13.045),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));

        public static final Pose2d rReef0 = new Pose2d(
                        Meters.of(13.876),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(180));
        public static final Pose2d rReef60 = new Pose2d(
                        Meters.of(13.46),
                        Meters.of(4.746),
                        Rotation2d.fromDegrees(-60));
        public static final Pose2d rReef120 = new Pose2d(
                        Meters.of(12.629),
                        Meters.of(4.746),
                        Rotation2d.fromDegrees(-120));
        public static final Pose2d rReef180 = new Pose2d(
                        Meters.of(12.213),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d rReef240 = new Pose2d(
                        Meters.of(12.629),
                        Meters.of(3.306),
                        Rotation2d.fromDegrees(60));
        public static final Pose2d rReef300 = new Pose2d(
                        Meters.of(13.461),
                        Meters.of(3.306),
                        Rotation2d.fromDegrees(120));

        public static final Pose2d rBranchA = new Pose2d(
                        Meters.of(13.876),
                        Meters.of(3.861),
                        Rotation2d.fromDegrees(180.000));
        public static final Pose2d rBranchB = new Pose2d(
                        Meters.of(13.876),
                        Meters.of(4.191),
                        Rotation2d.fromDegrees(180.000));
        public static final Pose2d rBranchC = new Pose2d(
                        Meters.of(13.603),
                        Meters.of(4.664),
                        Rotation2d.fromDegrees(240.000));
        public static final Pose2d rBranchD = new Pose2d(
                        Meters.of(13.318),
                        Meters.of(4.828),
                        Rotation2d.fromDegrees(240.000));
        public static final Pose2d rBranchE = new Pose2d(
                        Meters.of(12.771),
                        Meters.of(4.828),
                        Rotation2d.fromDegrees(300.000));
        public static final Pose2d rBranchF = new Pose2d(
                        Meters.of(12.486),
                        Meters.of(4.664),
                        Rotation2d.fromDegrees(300.000));
        public static final Pose2d rBranchG = new Pose2d(
                        Meters.of(12.213),
                        Meters.of(4.191),
                        Rotation2d.fromDegrees(0.000));
        public static final Pose2d rBranchH = new Pose2d(
                        Meters.of(12.213),
                        Meters.of(3.861),
                        Rotation2d.fromDegrees(0.000));
        public static final Pose2d rBranchI = new Pose2d(
                        Meters.of(12.486),
                        Meters.of(3.388),
                        Rotation2d.fromDegrees(60.000));
        public static final Pose2d rBranchJ = new Pose2d(
                        Meters.of(12.771),
                        Meters.of(3.223),
                        Rotation2d.fromDegrees(60.000));
        public static final Pose2d rBranchK = new Pose2d(
                        Meters.of(13.318),
                        Meters.of(3.223),
                        Rotation2d.fromDegrees(120.000));
        public static final Pose2d rBranchL = new Pose2d(
                        Meters.of(13.603),
                        Meters.of(3.388),
                        Rotation2d.fromDegrees(120.000));

        public static final Pose2d rAlgaeLeft = new Pose2d(
                        Meters.of(16.315),
                        Meters.of(2.197),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d rAlgaeMiddle = new Pose2d(
                        Meters.of(16.315),
                        Meters.of(4.026),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d rAlgaeRight = new Pose2d(
                        Meters.of(16.315),
                        Meters.of(5.855),
                        Rotation2d.fromDegrees(0));

        public static final Pose2d rCageProcessor = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(5.078),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d rCageMiddle = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(6.169),
                        Rotation2d.fromDegrees(0));
        public static final Pose2d rCageTable = new Pose2d(
                        Meters.of(8.76),
                        Meters.of(7.26),
                        Rotation2d.fromDegrees(0));

        public static final List<Pose2d> rReefFaces = Arrays.asList(
                        rReefCenter,
                        rReef0,
                        rReef60,
                        rReef120,
                        rReef180,
                        rReef240,
                        rReef300);

        public static final Map<ReefFace, Pose2d> rReef = Map.of(
                        ReefFace.CENTER, rReefCenter,
                        ReefFace.ZERO, rReef0,
                        ReefFace.SIXTY, rReef60,
                        ReefFace.ONETWENTY, rReef120,
                        ReefFace.ONEEIGHTY, rReef180,
                        ReefFace.TWOFOURTY, rReef240,
                        ReefFace.THREEHUNDRED, rReef300);

        public static final List<Pose2d> rBranchList = Arrays.asList(
                        rBranchA,
                        rBranchB,
                        rBranchC,
                        rBranchD,
                        rBranchE,
                        rBranchF,
                        rBranchG,
                        rBranchH,
                        rBranchI,
                        rBranchJ,
                        rBranchK,
                        rBranchL);

        public static final Map<Pose2d, Map<ReefBranchOffset, Pose2d>> rReefPoseSets = Map.of(
                        rReef0, Map.of(
                                        ReefBranchOffset.LEFT, rBranchA,
                                        ReefBranchOffset.MIDDLE, rReef0,
                                        ReefBranchOffset.RIGHT, rBranchB),
                        rReef60, Map.of(
                                        ReefBranchOffset.LEFT, rBranchC,
                                        ReefBranchOffset.MIDDLE, rReef60,
                                        ReefBranchOffset.RIGHT, rBranchD),
                        rReef120, Map.of(
                                        ReefBranchOffset.RIGHT, rBranchE,
                                        ReefBranchOffset.MIDDLE, rReef120,
                                        ReefBranchOffset.LEFT, rBranchF),
                        rReef180, Map.of(
                                        ReefBranchOffset.RIGHT, rBranchG,
                                        ReefBranchOffset.MIDDLE, rReef180,
                                        ReefBranchOffset.LEFT, rBranchH),
                        rReef240, Map.of(
                                        ReefBranchOffset.RIGHT, rBranchI,
                                        ReefBranchOffset.MIDDLE, rReef240,
                                        ReefBranchOffset.LEFT, rBranchJ),
                        rReef300, Map.of(
                                        ReefBranchOffset.LEFT, rBranchK,
                                        ReefBranchOffset.MIDDLE, rReef300,
                                        ReefBranchOffset.RIGHT, rBranchL));

        public static final Map<AlgaeType, Pose2d> rAlgaeType = Map.of(
                        AlgaeType.LEFT_ALGAE, bAlgaeLeft,
                        AlgaeType.MIDDLE_ALGAE, bAlgaeMiddle,
                        AlgaeType.RIGHT_ALGAE, bAlgaeRight);

        public static final Distance rAutoLineX = Meters.of(6.137 + fieldCenterOffset.getX());

        public static final HashMap<BranchID, Pose2d> rBranchMap = new HashMap<>();

        static {
                bBranchMap.put(BranchID.A, bBranchA);
                bBranchMap.put(BranchID.B, bBranchB);
                bBranchMap.put(BranchID.C, bBranchC);
                bBranchMap.put(BranchID.D, bBranchD);
                bBranchMap.put(BranchID.E, bBranchE);
                bBranchMap.put(BranchID.F, bBranchF);
                bBranchMap.put(BranchID.G, bBranchG);
                bBranchMap.put(BranchID.H, bBranchH);
                bBranchMap.put(BranchID.I, bBranchI);
                bBranchMap.put(BranchID.J, bBranchJ);
                bBranchMap.put(BranchID.K, bBranchK);
                bBranchMap.put(BranchID.L, bBranchL);

                rBranchMap.put(BranchID.A, rBranchA);
                rBranchMap.put(BranchID.B, rBranchB);
                rBranchMap.put(BranchID.C, rBranchC);
                rBranchMap.put(BranchID.D, rBranchD);
                rBranchMap.put(BranchID.E, rBranchE);
                rBranchMap.put(BranchID.F, rBranchF);
                rBranchMap.put(BranchID.G, rBranchG);
                rBranchMap.put(BranchID.H, rBranchH);
                rBranchMap.put(BranchID.I, rBranchI);
                rBranchMap.put(BranchID.J, rBranchJ);
                rBranchMap.put(BranchID.K, rBranchK);
                rBranchMap.put(BranchID.L, rBranchL);

        }

        /**
         * Checks if the current alliance is red. defaults to blue if no alliance is set
         * 
         * @return true if current alliance is red. false if current alliance is blue or
         *         not set
         */
        public static boolean isRedAlliance() {
                return DriverStation.getAlliance().isPresent() &&
                                DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }

        /**
         * Gets the pose of the amp for the current alliance
         * 
         * @return pose of the amp for the current alliance
         */
        public static Pose2d getAlgaeType(AlgaeType type) {
                return isRedAlliance() ? rAlgaeType.get(type) : bAlgaeType.get(type);
        }

        /**
         * Gets the pose of the stage for the current alliance
         * 
         * @param face face to retrieve
         * @return pose of the stage for the current alliance
         */
        public static Pose2d getReef(ReefFace face) {
                return isRedAlliance() ? rReef.get(face) : bReef.get(face);
        }

        /**
         * Get a list of all the reef face poses
         * 
         * @return list if all reef face poses
         */
        public static List<Pose2d> getReefList() {
                return isRedAlliance() ? rReefFaces : bReefFaces;
        }

        public static Map<BranchID, Pose2d> getBranchMap() {
                return isRedAlliance() ? rBranchMap : bBranchMap;
        }

        /**
         * Gets the Reef face pose sets for the current alliance
         * 
         * @return the Reef face pose sets for the current alliance
         */
        public static Map<Pose2d, Map<ReefBranchOffset, Pose2d>> getReefPoseSets() {
                return isRedAlliance() ? rReefPoseSets : bReefPoseSets;
        }

        /**
         * Gets the pose of the nearest reef face for the current alliance
         * 
         * @param pos current position
         * @return pose of the nearest reef face for the current alliance
         */
        public static Pose2d getNearestReefFace(Pose2d pose) {
                return pose.nearest(getReefList());
        }

        /**
         * Gets the set of poses for the reef face center nearest a given pose for the
         * current alliance
         * 
         * @param pose current robot pose
         * @return the set of poses for the reef face center nearest a given pose for
         *         the current alliance
         */
        public static Map<ReefBranchOffset, Pose2d> getNearestReefFaceBranch(Pose2d pose) {
                return getReefPoseSets().get(getNearestReefFace(pose));
        }

        /**
         * Gets the pose for scoring on a given branch
         * 
         * @param branch id of the branch to find
         * @return pose for scoring on the selected branch
         */
        public static Pose2d getBranch(BranchID branch) {
                return getBranchMap().get(branch);
        }

        /**
         * Gets the scoring pose for the nearest branch to a given pose
         * 
         * @param pose selected pose
         * @return scoring pose for the nearest branch to the given pose
         */
        public static Pose2d getNearestBranch(Pose2d pose) {
                return pose.nearest(isRedAlliance() ? rBranchList : bBranchList);
        }

        /**
         * Gets the field position of an algae with an offset applied
         * 
         * @param algaeType desired algae
         * @param offset    offset to apply
         * @return translation to the target algae with offset applied
         */
        public static Translation2d getAlgaeOffset(AlgaeType algaeType, Translation2d offset) {
                var algaePos = getAlgaeType(algaeType);

                return algaePos.getTranslation().plus(offset);
        }

        /**
         * Gets the x position of the autoline for the current alliance
         * 
         * @return x position of the autoline for the current alliance
         */
        public static Distance getAutoLineX() {
                return isRedAlliance() ? rAutoLineX : bAutoLineX;
        }

        /**
         * Gets the x position that will ensure the robot is clear of the auto zone line
         * 
         * @return x position that will ensure the robot is clear of the auto zone line
         */
        public static Distance getAutoClearX() {
                MutDistance distance = rAutoLineX.mutableCopy();

                if (isRedAlliance()) {
                        distance.minus(Constants.fullDiag).minus(Constants.autoClearance);
                } else {
                        distance.plus(Constants.fullDiag).plus(Constants.autoClearance);
                }

                return distance;
        }

        /**
         * Gets the forward angle for the robot for the current alliance
         * 
         * @return forward angle for the robot for the current alliance
         */
        public static Rotation2d getForwardAngle() {
                return isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        }

}