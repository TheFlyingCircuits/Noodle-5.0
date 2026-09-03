package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.PlayingField.FieldConstants;;

//perform an experiment.  Place the robot on the field.  Collect location data (x,y, theta).  Use the statistic tab in advandave scope to calculate the standard deviation
//This is an approximation due to the limitations of the roborio, determine the relationship between distance and standard deviation.  There is typically a plateu where the jitteryness increases dramatically
//The only thing we take into accound is that the spread of the data increases at larger distances
//Plot standard deviation and distance in desmos

public record SingleTagPoseObservation (String camName, Pose3d robotPose, double timestampSeconds, int tagUsed, double tagToCamMeters, double ambiguity, boolean usingMultiTag) {
    public Matrix<N3, N1> getStandardDeviations(boolean isScoringInHub) {
        // double slopeStdDevMeters_PerMeter = 0.0023;
        double slopeStdDevMeters_PerMeter;
        if(isScoringInHub) {
            slopeStdDevMeters_PerMeter = 0.0;
        } else if(DriverStation.isAutonomous()) {
            slopeStdDevMeters_PerMeter = 0.004;
        } else {
            slopeStdDevMeters_PerMeter = 0.010;
            if (tagToCamMeters < 1.5) {
                slopeStdDevMeters_PerMeter = 0.005;
            } else if (tagToCamMeters < 3) {
                slopeStdDevMeters_PerMeter = 0.008;
            }
        }

        return VecBuilder.fill(
            slopeStdDevMeters_PerMeter*tagToCamMeters,
            slopeStdDevMeters_PerMeter*tagToCamMeters,
            99999
        );
    }

    public Pose3d getTagPose() {
        return FieldConstants.tagPose(tagUsed);
    }
}
