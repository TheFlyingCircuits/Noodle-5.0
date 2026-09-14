package frc.robot.subsystems.drivetrain.drivetrain_util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.vision.SingleTagCam;
import frc.robot.subsystems.vision.SingleTagPoseObservation;

public class Odometry {

    Drivetrain drivetrain;

    public GyroIO gyroIO;
    public GyroIOInputsAutoLogged gyroInputs;

    private SingleTagCam[] tagCams = {
        new SingleTagCam(VisionConstants.tagCameraNames[0], VisionConstants.tagCameraTransforms[0]), // front
    };

    private boolean fullyTrustVisionNextPoseUpdate = false;
    private boolean allowTeleportsNextPoseUpdate = false;
    private boolean hasAcceptablePoseObservationsThisLoop = false;
    private Optional<FieldElement> focus = Optional.empty();

    
    public SwerveDrivePoseEstimator fusedPoseEstimator;
    public SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    public Odometry(Drivetrain drivetrain, GyroIO gyroIO) {
        this.drivetrain=drivetrain;

        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        gyroIO.setRobotYaw(0);

        // corresponds to x, y, and rotation standard deviations (meters and radians)
        // TODO: could use this experiment as a way to measure these, or we could just
        //       assume their values are fairly representative of what we'd get:
        //       https://docs.advantagekit.org/theory/high-frequency-odometry
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            drivetrain.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            gyroInputs.robotYawRotation2d,
            drivetrain.getModulePositions(), 
            new Pose2d());
    }

    //**************** ODOMETRY ****************//

    public void setPoseMeters(Pose2d pose) {
        fusedPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, drivetrain.getModulePositions(), pose);
        wheelsOnlyPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, drivetrain.getModulePositions(), pose);
    }
    public void setOrientation(Rotation2d orientation) {
        // keep location the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(currentPose.getTranslation(), orientation));
    }
    public void setLocation(Translation2d locationOnField) {
        // keep orientation the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(locationOnField, currentPose.getRotation()));
    }

     /**
     * Gets the current position of the robot on the field in meters, 
     * based off of our odometry and vision estimation.
     * This value considers the origin to be the right side of the blue alliance.
     * <p>
     * A positive X value brings the robot towards the red alliance, and a positive Y value
     * brings the robot towards the left side as viewed from the blue alliance.
     * <p>
     * Rotations are discontinuous counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * 
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the rotation reported by the gyro.
     * This rotation is continuous and counterclockwise positive.
     * 
     * This is not necessarily equivalent to the one reported by getPoseMeters(), and it is recommended
     * to use that rotation in almost every case.
     * 
     * This is usable for calibrating the wheel radii, where a continuous angle is required.
     * @return
     */
    public Rotation2d getGyroRotation2d() {
        return gyroInputs.robotYawRotation2d;
    }

    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Rotation2d forwardOnRed = Rotation2d.k180deg;
        Rotation2d forwardOnBlue = Rotation2d.kZero;
        Rotation2d forwardNow = getPoseMeters().getRotation();
        this.setOrientation(FlyingCircuitUtils.getAllianceDependentValue(forwardOnRed, forwardOnBlue, forwardNow));
    }

    /** Makes the pose estimator only use tags that are on the given field element. */
    public void setFocus(FieldElement focus) {
        this.focus = Optional.of(focus);
    }
    /** Allows the pose estimator to use all apriltags on the field, instead of only those attached to a specific field element. */
    public void resetFocus() {
        this.focus = Optional.empty();
    }
    public void fullyTrustVisionNextPoseUpdate() {
        this.fullyTrustVisionNextPoseUpdate = true;
    }
    public void allowTeleportsNextPoseUpdate() {
        this.allowTeleportsNextPoseUpdate = true;
    }
    public boolean seesAcceptableTag() {
        return this.hasAcceptablePoseObservationsThisLoop;
    }

    
    public void updatePoseEstimator() {
        // setFocus(FieldElement.HUB);
        // log flags that were set in between last pose update and now
        Logger.recordOutput("drivetrain/fullyTrustingVision", this.fullyTrustVisionNextPoseUpdate);
        Logger.recordOutput("drivetrain/allowingPoseTeleports", this.allowTeleportsNextPoseUpdate);

        // update with wheel deltas
        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, drivetrain.getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, drivetrain.getModulePositions());

        // get all pose observations from each camera
        List<SingleTagPoseObservation> allFreshPoseObservations = new ArrayList<>();
        for (SingleTagCam tagCam : tagCams) {
            allFreshPoseObservations.addAll(tagCam.getFreshPoseObservations(false, getPoseMeters().getRotation().getDegrees()));
        }

        // process pose obvervations in chronological order
        allFreshPoseObservations.sort(new Comparator<SingleTagPoseObservation>() {
            public int compare(SingleTagPoseObservation a, SingleTagPoseObservation b) {
                return Double.compare(a.timestampSeconds(), b.timestampSeconds());
            } 
        });

        // Filter tags
        List<Pose3d> acceptedTags = new ArrayList<>();
        List<Pose3d> rejectedTags = new ArrayList<>();
        for (SingleTagPoseObservation poseObservation : allFreshPoseObservations) {

            Translation2d observedLocation = poseObservation.robotPose().getTranslation().toTranslation2d();
            Translation2d locationNow = getPoseMeters().getTranslation();

            // reject tags that are too far away
            if (poseObservation.tagToCamMeters() > 6.0) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // reject pose observations that claim the robot
            // is in the air or beneath the floor
            if (Math.abs(poseObservation.robotPose().getZ()) > Units.inchesToMeters(7)) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // reject tags that are too ambiguous
            if (poseObservation.ambiguity() > 0.25) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // Don't allow the robot to teleport. Disallowing teleports can cause problems when we get bumped
            // and experience lots of wheel slip, which is why we have the "allowTeleportsNextPoseUpdate" flag
            // (used at driver's discretion (typically via y-button)). Also useful for seeding the robot pose
            // at the beginning of a match.
            double teleportToleranceMeters = 2.0;
            if ((observedLocation.getDistance(locationNow) > teleportToleranceMeters) && (!this.allowTeleportsNextPoseUpdate)) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // Don't use tags that are irrelevant to our current goal (e.g. only use hub tags when shooting).
            if ((focus.isPresent() && !focus.get().hasTagID(poseObservation.tagUsed()))) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // This measurment passes all our checks, so we add it to the fusedPoseEstimator
            acceptedTags.add(poseObservation.getTagPose());
            Matrix<N3, N1> stdDevs = this.fullyTrustVisionNextPoseUpdate ? VecBuilder.fill(0, 0, 0) : poseObservation.getStandardDeviations((focus.isPresent() && focus.get() == FieldElement.HUB));

            fusedPoseEstimator.addVisionMeasurement(
                poseObservation.robotPose().toPose2d(), 
                poseObservation.timestampSeconds(), 
                stdDevs
            );
        }

        // reset flags for next time
        this.fullyTrustVisionNextPoseUpdate = false;
        this.allowTeleportsNextPoseUpdate = false;
        this.hasAcceptablePoseObservationsThisLoop = acceptedTags.size() > 0;

        // log the accepted and rejected tags
        Logger.recordOutput("drivetrain/acceptedTags", acceptedTags.toArray(new Pose3d[0]));
        Logger.recordOutput("drivetrain/rejectedTags", rejectedTags.toArray(new Pose3d[0]));
        
        allFreshPoseObservations = null;
    }


    public Optional<Translation3d> getClosestCluster() {
        return null;
        // return intakeCam.getClosestClusterTo(getPoseMeters().getTranslation());
    }
}
