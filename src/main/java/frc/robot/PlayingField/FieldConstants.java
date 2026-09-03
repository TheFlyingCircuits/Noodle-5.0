package frc.robot.PlayingField;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // See Table 5-2 in Game Manual
    public static final double maxX = tagLayout.getFieldLength();
    public static final double maxY = tagLayout.getFieldWidth();
    public static final Translation2d midField = new Translation2d(maxX / 2.0, maxY / 2.0);

    public static Pose3d tagPose(int tagID) {
        return tagLayout.getTagPose(tagID).get();
    }

    public static Transform3d tagPoseAsTransform(int tagID) {
        Pose3d pose = tagLayout.getTagPose(tagID).get();
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Returns true if the fed position is inside the field perimeter.
     * @param toleranceMeters - How far the given coordinates can poke outside of the
     *                          field perimeter, while still having this function return true.
     */
    public static boolean isInField(Translation2d location, double toleranceMeters) {
        boolean insideX = ((0 - toleranceMeters) < location.getX()) && (location.getX() < (maxX + toleranceMeters));
        boolean insideY = ((0 - toleranceMeters) < location.getY()) && (location.getY() < (maxY + toleranceMeters));
        return insideX && insideY;
    }
    /** Returns true if the fed position is inside the field perimeter. No tolerance is applied. */
    public static boolean isInField(Translation2d location) {
        return isInField(location, 0);
    }


    // Fuel geometry
    public static final double fuelDiameterMeters = 0.15; // Game Manual seciton 5.10
    public static final double fuelRadiusMeters = fuelDiameterMeters / 2.0;

    // Hub geometry
    public static final double hubBaseWidthMeters = Units.inchesToMeters(47.0);
    public static final double halfHubBaseWidthMeters = hubBaseWidthMeters / 2.0;
    /** Distance from the middle of the field to the center of the hub, as measured along the field's x-axis. */
    public static final double midFieldToHubMeters = Units.inchesToMeters(120.0) + halfHubBaseWidthMeters;

    // The fuel enters the hub through an inverted hexagonal pyramid, which I refer to as "the funnel".
    //
    // The upperHubFunnel refers to the widest part of the inverted pyramid
    // (i.e. the part that's furthest from the ground).
    //
    // The lowerHubFunnel refers to the narrowest part of the inverted pyramid
    // (i.e. where the pyramid joins up to the rest of the hub. See GE-26315 in the field drawings).
    public static final double upperHubFunnel_heightMeters = Units.inchesToMeters(72.0);
    public static final double upperHubFunnel_centerToEdgeMeters = Units.inchesToMeters(41.73 / 2.0);
    public static final double upperHubFunnel_centerToCornerMeters = upperHubFunnel_centerToEdgeMeters / Math.cos(Math.PI / 6.0);

    public static final double lowerHubFunnel_heightMeters = Units.inchesToMeters(56.440945); // Measured in official field CAD because I couldn't find this in the field drawings anywhere.
    public static final double lowerHubFunnel_centerToEdgeMeters = Units.inchesToMeters(25.16-1.84) / 2.0;
    public static final double lowerHubFunnel_centerToCornerMeters = Units.inchesToMeters(28.47-1.53) / 2.0;

    // Net geometry
    /** Meters from the floor to the top of the net. It's about the same height as an NBA basketball hoop. */
    public static final double netTopZ = Units.inchesToMeters(120.36);
    /** Meters from the floor to the bottom of the net. */
    public static final double netBottomZ = Units.inchesToMeters(49.75);
    /** The extent of the net along the field's y-axis. */
    public static final double netWidthMeters = Units.inchesToMeters(58.41);
    /** Distance from the center of the hub to the net, as measured along the field's x-axis. */
    public static final double hubToNetMeters = halfHubBaseWidthMeters + Units.inchesToMeters(10.26);

    // Bump geometry
    /** The extent of the bump along the field's y-axis */
    public static final double bumpWidthMeters = Units.inchesToMeters(73.0);
    public static final double bumpHeightMeters = Units.inchesToMeters(6.56);
    public static final double bumpInclineDegrees = 15.0;
    /** Distance from the center of the hub to the center of the bump, as measured along the field's y-axis. */
    public static final double hubToBumpMeters = halfHubBaseWidthMeters + (bumpWidthMeters / 2.0);

    // Trench geometry
    /** The extent of the trench along the field's y-axis. */
    public static final double trenchWidthMeters = midField.getY() - halfHubBaseWidthMeters - bumpWidthMeters - Units.inchesToMeters(12); // Computing the trenchWidth in this way should automatically account for differences between the "welded" and "andymark" field perimeters.
    public static final double trenchHeightMeters = Units.inchesToMeters(22.25);
    public static final double verticalExtensionLimitMeters = Units.inchesToMeters(30);
    /** Distance from the center of the hub to the center of the trench, as measured along the field's y-axis. */
    public static final double hubToTrenchMeters = halfHubBaseWidthMeters + bumpWidthMeters + Units.inchesToMeters(12.0) + (trenchWidthMeters / 2.0);

    // Depot geometry
    /** The extent of the depot along the field's x-axis. */
    public static final double depotDepthIntoFieldMeters = Units.inchesToMeters(27.0);
    /** The extent of the depot along the field's y-axis. */
    public static final double depotWidthMeters = Units.inchesToMeters(42.0);
    /** From the driver's perspective, this is how far left the center of the depot is from the center of the field. */
    public static final double depotLateralOffsetMeters = Units.inchesToMeters(75.93);



    /** Used for photon vision simulation. See https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html */
    public static final VisionSystemSim simulatedTagLayout = new VisionSystemSim("simulatedTagLayout");
    public static final VisionSystemSim simulatedFuelLayout = new VisionSystemSim("simulatedFuelLayout");
    static {
        simulatedTagLayout.addAprilTags(FieldConstants.tagLayout);

        // Add some fuel in the center of the field

        // The same for all fuel
        TargetModel simulatedFuelShape = new TargetModel(fuelDiameterMeters);

        // Can be different for different fuel
        Pose3d simulatedFuelPose = new Pose3d(midField.getX(), midField.getY(), fuelRadiusMeters, Rotation3d.kZero);

        // Add to the layout
        simulatedFuelLayout.addVisionTargets("fuel", new VisionTargetSim(simulatedFuelPose, simulatedFuelShape));
    }
}
