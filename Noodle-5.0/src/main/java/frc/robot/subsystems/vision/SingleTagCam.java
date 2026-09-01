package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.PlayingField.FieldConstants;

public class SingleTagCam {

    private final PhotonCamera cam;
    private Translation3d camLocation_robotFrame;
    private Rotation3d camOrientation_robotFrame;
    private boolean usingMultiTag = false;

    public SingleTagCam(String name, Translation3d camLocation_robotFrame, Rotation3d camOrientation_robotFrame) {
        cam = new PhotonCamera(name);
        this.camLocation_robotFrame = camLocation_robotFrame;
        this.camOrientation_robotFrame = camOrientation_robotFrame;

        // TODO: customize simSettings instead of using a preset
        //       (not strictly necessary, but could be nice).
        SimCameraProperties simSettings = SimCameraProperties.PI4_LIFECAM_640_480();
        PhotonCameraSim simCamera = new PhotonCameraSim(cam, simSettings);

        // minTargetAreaPixels was set based on the smallest apriltag we should probably detect.
        // IRL, the tags are squares made up of 100 tiles (36 data bit tiles, 28 tiles of black border
        // around the data bits, and then 36 tiles of white border around the black border).
        // Each individual tile on an apriltag should probably land on at least a 3x3 block
        // of pixels in the image plane to be detected reliably.
        int apriltagTiles = 100;
        int pixelsPerTile = 3 * 3;
        simCamera.setMinTargetAreaPixels(apriltagTiles * pixelsPerTile);
        // FieldConstants.simulatedTagLayout.addCamera(simCamera, new Transform3d(camLocation_robotFrame, camOrientation_robotFrame));
    }
    public SingleTagCam(String name, Pose3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }
    public SingleTagCam(String name, Transform3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }

    
    
    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingTrig, 
    double robotAngleDeg) {

        // calibrateCamPose_robotFrame();
        List<SingleTagPoseObservation> output = new ArrayList<>();

        // advantage scope viz hacks
        List<Pose3d> justRobotPoses = new ArrayList<>();
        List<Translation3d> sightlines = new ArrayList<>();

        // See if we've gotten any new frames since last time
        List<PhotonPipelineResult> freshFrames = cam.getAllUnreadResults();
        if (cam.isConnected()) {
            Logger.recordOutput("camConnections/"+cam.getName(), true);
        }
        else {
            Logger.recordOutput("camConnections/"+cam.getName(), false);
        }
        if (freshFrames.size() == 0) {
            return output;
        }

        // For each frame, get pose info for all tags seen in that frame.
        for (PhotonPipelineResult frame : freshFrames) {
            if(usingMultiTag) {
                if(frame.multitagResult.isPresent()) {
                    // Pose3d robotPose = this.getRobotPoseFromSingleTag(tag);
                    Transform3d cameraTransformEstimate = frame.multitagResult.get().estimatedPose.best;
                    Pose3d robotPose = getRobotPoseFromSingleMutliTag(cameraTransformEstimate);
                    double timestamp = frame.getTimestampSeconds();

                    List<Short> tagsUsed = frame.multitagResult.get().fiducialIDsUsed;
                    double tagToCamDistance = tagsUsed.get(0);
                    
                    for(Short tagID : tagsUsed) {
                        double currentTagDistance = Math.abs(robotPose.getTranslation().minus(
                            FieldConstants.tagPose(tagID).getTranslation()).getNorm());
                        if(currentTagDistance < tagToCamDistance) {
                            tagToCamDistance = currentTagDistance;
                        }
                    }
                    double ambiguity = 0.0;

                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(cam.getName(), robotPose, timestamp, 0, tagToCamDistance, ambiguity, true);
                    output.add(poseObservation);

                    // advantage scope viz
                    justRobotPoses.add(robotPose);
                    Pose3d camPoseOnfield = robotPose.plus(new Transform3d(camLocation_robotFrame, camOrientation_robotFrame));
                    int oneOfTheTags = tagsUsed.indexOf(0);
                    Pose3d tagPoseOnField = FieldConstants.tagPose(oneOfTheTags);
                    // TODO: Maybe add a segment that brings the line under the field, so we can hide
                    //       the little connection between different pose estimates?
                    sightlines.addAll(Arrays.asList(camPoseOnfield.getTranslation(), tagPoseOnField.getTranslation(), camPoseOnfield.getTranslation()));
                }
            } else {
                for (PhotonTrackedTarget tag : frame.targets) {
                    // Extract data from PhotonVision.
                    int tagID = tag.fiducialId;
                    Pose3d robotPose;
                    if(usingTrig) {
                        robotPose = this.getRobotPoseFromSingleTagTrig(tag, robotAngleDeg);
                    } else {
                        robotPose = this.getRobotPoseFromSingleTag(tag);
                    }

                    double timestamp = frame.getTimestampSeconds();
                    double tagToCamDistance = tag.getBestCameraToTarget().getTranslation().getNorm();
                    double ambiguity = tag.poseAmbiguity;

                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(cam.getName(), robotPose, timestamp, tagID, tagToCamDistance, ambiguity, false);
                    output.add(poseObservation);

                    // advantage scope viz
                    justRobotPoses.add(robotPose);
                    Pose3d camPoseOnfield = robotPose.plus(new Transform3d(camLocation_robotFrame, camOrientation_robotFrame));
                    Pose3d tagPoseOnField = FieldConstants.tagPose(tagID);
                    // TODO: Maybe add a segment that brings the line under the field, so we can hide
                    //       the little connection between different pose estimates?
                    sightlines.addAll(Arrays.asList(camPoseOnfield.getTranslation(), tagPoseOnField.getTranslation(), camPoseOnfield.getTranslation()));
                }
            }
        }

        // viz persists between robot loops with no freshly processed frames
        Logger.recordOutput("tagCams/"+cam.getName()+"/singleTagPoseObservations", output.toArray(new SingleTagPoseObservation[0]));
        Logger.recordOutput("tagCams/"+cam.getName()+"/mostRecentObservedPoses", justRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("tagCams/"+cam.getName()+"/mostRecentSightlines", sightlines.toArray(new Translation3d[0]));
        return output;
    }

    public String getName() {
        return cam.getName();
    }

    private Pose3d getRobotPoseFromSingleTagTrig(PhotonTrackedTarget singleTag, double robotAngleDeg) {
        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();


        Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
        new Rotation2d();
        // Transform3d sightlineCamToTag = new Transform3d()
        double trigangleThetaRad = Units.degreesToRadians
            (singleTag.pitch + Units.radiansToDegrees(camOrientation_robotFrame.getY()));

        double camToTagDistance = tagPose_fieldFrame.getZ() / Math.sin(trigangleThetaRad);

        Translation3d camTranslation = tagPose_fieldFrame.getTranslation().minus(
            new Translation3d(camToTagDistance, camToTagDistance,
            camToTagDistance*Math.sin(trigangleThetaRad))
        );


        return new Pose3d();
    }

    private Pose3d getRobotPoseFromSingleTag(PhotonTrackedTarget singleTag) {

        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();


        Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
        Transform3d camPose_tagFrame = singleTag.getBestCameraToTarget().inverse();
        Transform3d camPose_fieldFrame = tagPose_fieldFrame.plus(camPose_tagFrame);
        Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);

        return new Pose3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());
    }

    private Pose3d getRobotPoseFromSingleMutliTag(Transform3d resultTransform) {

        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();

        Transform3d camPose_fieldFrame = resultTransform;
        Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);

        return new Pose3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());
    }



    private void calibrateCamPose_robotFrame() {
        int calibrationTagID = 16; // random placeholder for now

        double frameXDistanceFrameTag = Units.inchesToMeters(27.5);
        // double tagX_robotFrame = (DrivetrainConstants.frameWidthMeters/2.0) + frameXDistanceFrameTag;
        double tagX_robotFrame = -frameXDistanceFrameTag;
        double tagY_robotFrame = 0.0;
        double tagZ_robotFrame = Units.inchesToMeters(26.0625); // this will be prob be the height of the calibration tag

        // boolean facingTag = FlyingCircuitUtils.getBooleanFromDashboard("facingCalibrationTag", false);
        Rotation3d tagOrientation_robotFrame = new Rotation3d(Rotation2d.k180deg.minus(Rotation2d.k180deg)); // change based off cam
        // if (facingTag) {
        //     tagOrientation_robotFrame = new Rotation3d(Rotation2d.k180deg);
        // }
        this.calibrateCamPose_robotFrame(calibrationTagID, new Pose3d(tagX_robotFrame, tagY_robotFrame, tagZ_robotFrame, tagOrientation_robotFrame));
    }

    /** When paired with a measurement jig, this function helps us determine the location/orientaiton
     *  of the cameras on our robot without having to make awkward measurements.
     */
    private void calibrateCamPose_robotFrame(int tagToLookFor, Pose3d tagPose_robotFrame) {
        // Check for new frames from the camera
        List<PhotonPipelineResult> newFrames = cam.getAllUnreadResults();
        if (newFrames.size() == 0) {
            return;
        }

        // Iterate through all seen tags from the most recent frame until we find the calibration tag
        PhotonPipelineResult mostRecentFrame = newFrames.get(newFrames.size()-1);
        Transform3d tagPose_camFrame = null;
        for (PhotonTrackedTarget tag : mostRecentFrame.targets) {
            if (tag.getFiducialId() == tagToLookFor) {
                tagPose_camFrame = tag.getBestCameraToTarget();
                break;
            }
        }

        // Early exit if we didn't see the calibration tag.
        if (tagPose_camFrame == null) { return; }

        // Reverse engineer the camera's placement
        Transform3d camPose_tagFrame = tagPose_camFrame.inverse();
        Transform3d robotPose_tagFrame = new Transform3d(tagPose_robotFrame.getTranslation(), tagPose_robotFrame.getRotation()).inverse();
        Pose3d camPose_robotFrame = Pose3d.kZero.plus(camPose_tagFrame).relativeTo(Pose3d.kZero.plus(robotPose_tagFrame));

        // Report results to AdvantageScope, and we're done!
        Logger.recordOutput("cameraPlacementCalibration/"+cam.getName()+"/pose", camPose_robotFrame);
        Logger.recordOutput("cameraPlacementCalibration/"+cam.getName()+"/rollDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getX()));
        Logger.recordOutput("cameraPlacementCalibration/"+cam.getName()+"/pitchDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getY()));
        Logger.recordOutput("cameraPlacementCalibration/"+cam.getName()+"/yawDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getZ()));

        // Note to self: The intent of this function is to let it run and collect data for a minute or so to collect a bunch of data points,
        //               and then use AdvantageScope's statistics feature to get an average value for the camera placement.
        //               However, I know from past experience that averaging angular quantities can be tricky, or sometimes even ill-defined!
        //               For example, should the average of 0 degrees and 360 degrees be 180 degrees? In one sense, the average of a full turn
        //               and not turning at all is indeed a half turn. But in another sense, the average should be 0 (or 360) because 0 and 360
        //               are the same in terms of describing a direction / orientation. This second sense is probably closer to what we want
        //               for pose estimation, because we're mostly consered with how we're oriented right now, rather than how exactly we got
        //               to that orientaiton. One method I read about online for computing "angluar / circular averages" in this second sense is
        //               to simply average the direction vectors themselves, rather than their angles. However, this doesn't work in 3D, because
        //               averaging each column of many rotation matricies to yield an [avgXHat, avgYHat, avgZHat] doesn't necessarily yield a set
        //               of 3 orthogonal vectors! This seems like an interesting rabbit hole to dive into later, but I unfortuantley don't have
        //               the time at the moment. If I do get some time later though, one thing I want to look into weighted slerping
        //               (i.e. slerp between A and B to get the average of 2 orientations, then slerp 2/3rds of the way from that average towards orientation C, and so on.)
    }


}

