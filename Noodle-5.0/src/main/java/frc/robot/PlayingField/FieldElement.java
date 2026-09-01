package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FlyingCircuitUtils;

public enum FieldElement {
    // A standard FieldElement with no extra bells or whistles.
    // Just a name, and some alliance-aware pose/tag info.
    HUB(new int[] {10, 2, 4, 5}, new int[] {9, 11, 3, 8}, new int[] {26, 18, 20, 21}, new int[] {25, 27, 19, 24}),
    Enemy_HUB(new int[] {26, 18, 20, 21}, new int[] {25, 27, 19, 24, 22, 23, 17, 28}, new int[] {10, 2, 4, 5}, new int[] {9, 11, 3, 8, 6, 7, 1, 12}),
    TRENCH_LEFT(new int[] {6, 7}, new int[0], new int[] {22, 23}, new int[0]),
    TRENCH_RIGHT(new int[] {1, 12}, new int[0], new int[] {17, 28}, new int[0]),
    OUTPOST(new int[] {13}, new int[] {14}, new int[] {29}, new int[] {30}),
    TOWER(new int[] {15}, new int[] {16}, new int[] {31}, new int[] {32}),
    DEPOT("DEPOT"),
    BUMP_LEFT("BUMP_LEFT"), BUMP_RIGHT("BUMP_RIGHT");

    private final Pose3d redPose;
    private final Pose3d bluePose;
    private final int[] redTagIDs;
    private final int[] blueTagIDs;

    private FieldElement(int[] primaryRedTagIDs, int[] secondaryRedTagIDs, int[] primaryBlueTagIDs, int[] secondaryBlueTagIDs) {
        // Many field elements this year have "primary" tags which are centered on the field element,
        // and "secondary" tags which are offset on the field element. Poses will be based only
        // on the primary tagIDs, but we also need to know the secondary tagIDs so we can have a list of
        // all tags that are associated with each element.
        Translation3d redLocation = FieldElement.getAvgLocation(primaryRedTagIDs);
        Translation3d blueLocation = FieldElement.getAvgLocation(primaryBlueTagIDs);

        // The orientation component of each field element doesn't really have a clearly defined value
        // this year, as we're likely to interact with each element from a variety of angles. As such,
        // I've arbitrarily chosen to just have each field element "face" the opposing alliance wall
        // (i.e. forward from a driver's perspective), but we can revisit this later if needed.
        this.bluePose = new Pose3d(blueLocation, Rotation3d.kZero);
        this.redPose = new Pose3d(redLocation, new Rotation3d(Rotation2d.k180deg));

        this.redTagIDs = FieldElement.mergeIntArrays(primaryRedTagIDs, secondaryRedTagIDs);
        this.blueTagIDs = FieldElement.mergeIntArrays(primaryBlueTagIDs, secondaryBlueTagIDs);
    }

    private FieldElement(String name) {
        // An alternate constructor for field elements that don't have any tags associated with them.

        if (name.equals("DEPOT")) {
            // Phrasing the depot's location in this way should account for both the weleded and andymark field perimeters.
            Translation3d redLocation = new Translation3d(FieldConstants.maxX, FieldConstants.midField.getY() - FieldConstants.depotLateralOffsetMeters, 0);
            Translation3d blueLocation = new Translation3d(0, FieldConstants.midField.getY() + FieldConstants.depotLateralOffsetMeters, 0);

            this.bluePose = new Pose3d(blueLocation, Rotation3d.kZero);
            this.redPose = new Pose3d(redLocation, new Rotation3d(Rotation2d.k180deg));

            this.redTagIDs = new int[0];
            this.blueTagIDs = new int[0];
        }
        else if (name.equals("BUMP_LEFT")) {
            double redX = FieldConstants.midField.getX() + FieldConstants.midFieldToHubMeters;
            double redY = FieldConstants.midField.getY() - FieldConstants.hubToBumpMeters;

            double blueX = FieldConstants.midField.getX() - FieldConstants.midFieldToHubMeters;
            double blueY = FieldConstants.midField.getY() + FieldConstants.hubToBumpMeters;

            this.bluePose = new Pose3d(blueX, blueY, FieldConstants.bumpHeightMeters, Rotation3d.kZero);
            this.redPose = new Pose3d(redX, redY, FieldConstants.bumpHeightMeters, new Rotation3d(Rotation2d.k180deg));

            this.redTagIDs= new int[0];
            this.blueTagIDs = new int[0];
        }
        else if (name.equals("BUMP_RIGHT")) {
            double redX = FieldConstants.midField.getX() + FieldConstants.midFieldToHubMeters;
            double redY = FieldConstants.midField.getY() + FieldConstants.hubToBumpMeters;

            double blueX = FieldConstants.midField.getX() - FieldConstants.midFieldToHubMeters;
            double blueY = FieldConstants.midField.getY() - FieldConstants.hubToBumpMeters;

            this.bluePose = new Pose3d(blueX, blueY, FieldConstants.bumpHeightMeters, Rotation3d.kZero);
            this.redPose = new Pose3d(redX, redY, FieldConstants.bumpHeightMeters, new Rotation3d(Rotation2d.k180deg));

            this.redTagIDs= new int[0];
            this.blueTagIDs = new int[0];
        }
        else {
            throw new RuntimeException("ERROR IN FieldElement.java! TRIED TO INSTANTIATE AN UNRECOGNIZED FIELD ELEMENT!!!");
        }
    }

    public Pose3d getPose() {
        return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
    }

    public Translation3d getLocation() {
        return this.getPose().getTranslation();
    }

    public Rotation3d getOrientation() {
        return this.getPose().getRotation();
    }

    public Pose2d getPose2d() {
        return this.getPose().toPose2d();
    }

    public Translation2d getLocation2d() {
        return this.getPose2d().getTranslation();
    }

    public Rotation2d getOrientation2d() {
        return this.getPose2d().getRotation();
    }

    public boolean hasTagID(int idToFind) {
        for (int id : FlyingCircuitUtils.getAllianceDependentValue(redTagIDs, blueTagIDs, new int[0])) {
            if (id == idToFind) {
                return true;
            }
        }
        return false;
    }

    private static Translation3d getAvgLocation(int[] tagsToSample) {
        Translation3d avgLocation = new Translation3d();
        for (int tagID : tagsToSample) {
            avgLocation = avgLocation.plus(FieldConstants.tagPose(tagID).getTranslation());
        }
        avgLocation = avgLocation.times(1.0 / tagsToSample.length);
        return avgLocation;
    }

    private static int[] mergeIntArrays(int[] a, int[] b) {
        int[] output = new int[a.length + b.length];

        int outputIndex = 0;
        for (int i = 0; i < a.length; i += 1) {
            output[outputIndex] = a[i];
            outputIndex += 1;
        }
        for (int i = 0; i < b.length; i += 1) {
            output[outputIndex] = b[i];
            outputIndex += 1;
        }

        return output;
    }

    // private Pose3d[] getTagPoses() {
    //     int[] tagIDs = FlyingCircuitUtils.getAllianceDependentValue(redTagIDs, blueTagIDs, new int[0]);
    //     ArrayList<Pose3d> tagPoses = new ArrayList<>();
    //     for (int id : tagIDs) {
    //         tagPoses.add(FieldConstants.tagPose(id));
    //     }

    //     return tagPoses.toArray(new Pose3d[0]);
    // }

    // public static void advantageScopeViz() {
    //     for (FieldElement element : FieldElement.values()) {
    //         Logger.recordOutput("FieldElementViz/"+element.name(), element.getPose());
    //         Logger.recordOutput("FieldElementViz/"+element.name()+"/tags", element.getTagPoses());
    //     }
    // }
}

