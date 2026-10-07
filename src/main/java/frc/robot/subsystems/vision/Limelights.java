package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Limelights {
    public ArrayList<String> camNames = new ArrayList<String>();
    public boolean shouldChangeIMUMode = false;
    public int imuMode = 0;

    public Limelights(ArrayList<String> camNames) {
        this.camNames=camNames;
        for(String camName : camNames) {
            LimelightHelpers.setPipelineIndex(camName, 0);
        }
    }

    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingMT2, Drivetrain drivetrain) {
        ArrayList<SingleTagPoseObservation> poseObservations = new ArrayList<SingleTagPoseObservation>();

        for(String camName : camNames) {
            LimelightHelpers.SetRobotOrientation(
                camName, drivetrain.odometry.getPoseMeters().getRotation().getDegrees(), 
                Units.radiansToDegrees(drivetrain.getRobotRelativeVelocityMPS().omegaRadiansPerSecond), 0, 0, 0, 0);

            if(usingMT2) {
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
        
                if(LimelightHelpers.validPoseEstimate(mt2)) {
                    RawFiducial[] fiducialsUsed = mt2.rawFiducials;
                    boolean seesMultibleTags = fiducialsUsed.length > 1;

                    RawFiducial closestTag = fiducialsUsed[0];
                    for(RawFiducial tag : fiducialsUsed) {
                        if(tag.distToCamera < closestTag.distToCamera) closestTag = tag;
                    }

                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
                        camName, new Pose3d(mt2.pose), mt2.timestampSeconds, fiducialsUsed, 
                            closestTag.distToCamera, 0.0, seesMultibleTags, closestTag.id);

                    poseObservations.add(poseObservation);
                }
            } else {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);
        
                if(LimelightHelpers.validPoseEstimate(mt1)) {
                    RawFiducial[] fiducialsUsed = mt1.rawFiducials;
                    boolean seesMultibleTags = fiducialsUsed.length > 1;

                    RawFiducial closestTag = fiducialsUsed[0];
                    for(RawFiducial tag : fiducialsUsed) {
                        if(tag.distToCamera < closestTag.distToCamera) closestTag = tag;
                    }

                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
                        camName, new Pose3d(mt1.pose), mt1.timestampSeconds, fiducialsUsed, 
                            closestTag.distToCamera, closestTag.ambiguity, seesMultibleTags, closestTag.id);

                    poseObservations.add(poseObservation);
                }
            }

        }

        // after we get poses and also set the robot pose for each cam change imu mode if queued
        if(shouldChangeIMUMode) {
            for(String camName : camNames) {
                LimelightHelpers.SetIMUMode(camName, imuMode);
            }
            shouldChangeIMUMode = false;
        }

        return poseObservations;
    }

    public void setIMUModeNextLoop(int mode) {
        imuMode = mode;
        shouldChangeIMUMode = true;
    } 

    public void setIMUModeNow(int mode) {
        for(String camName : camNames) {
            LimelightHelpers.SetIMUMode(camName, imuMode);
        }
    }

}
