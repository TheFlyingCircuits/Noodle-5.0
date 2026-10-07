package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.PlayingField.FieldConstants;;

//perform an experiment.  Place the robot on the field.  Collect location data (x,y, theta).  Use the statistic tab in advandave scope to calculate the standard deviation
//This is an approximation due to the limitations of the roborio, determine the relationship between distance and standard deviation.  There is typically a plateu where the jitteryness increases dramatically
//The only thing we take into accound is that the spread of the data increases at larger distances
//Plot standard deviation and distance in desmos

public record SingleTagPoseObservation (String camName, Pose3d robotPose, double timestampSeconds, RawFiducial[] tagsUsed, double tagToCamMeters, double ambiguity, boolean usingMultiTag, int closestTagID) {
    public Matrix<N3, N1> getStandardDeviations(boolean isScoringInHub) {

        if(usingMultiTag) {
            return VecBuilder.fill(
            0.01,
            0.01,
            0.05
        );
        
        }
        // double slopeStdDevMeters_PerMeter = 0.0023;
        double xyStdDev = 0.1 * Math.pow(tagToCamMeters, 2);

        double rotStdDev = 0.25 * Math.pow(tagToCamMeters, 2);

        return VecBuilder.fill(
            xyStdDev,
            xyStdDev,
            rotStdDev
        );
    }

    public Pose3d getTagPose() {
        return FieldConstants.tagPose(closestTagID);
    }
}
