package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionTargetSim;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.subsystems.drivetrain.drivetrain_util.Odometry;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.swerve.SwerveModule;
import frc.robot.subsystems.drivetrain.swerve.SwerveModuleIO;

public class Drivetrain extends SubsystemBase {

    private SwerveModule[] swerveModules;

    public Odometry odometry;

    private final PIDController xController = new PIDController(6, 0.1, 0.0);
    private final PIDController yController = new PIDController(6, 0.1, 0.0);
    private final PIDController headingController = new PIDController(2.5, 0.0, 0.0);

    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO
    ) {

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0, "frontLeft"),
            new SwerveModule(frSwerveModuleIO, 1, "frontRight"),
            new SwerveModule(blSwerveModuleIO, 2, "backLeft"),
            new SwerveModule(brSwerveModuleIO, 3, "backRight")
        };

        this.odometry = new Odometry(this, gyroIO);
         Logger.recordOutput("choreo target pose", new Pose2d(0, 0, new Rotation2d()));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxDesiredTeleopVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            swervePositions[mod.moduleIndex] = mod.getPosition();
        }

        return swervePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            swerveStates[mod.moduleIndex] = mod.getState();
        }

        return swerveStates;
    }

    //**************** DRIVING ****************//

    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void robotOrientedDrive(ChassisSpeeds desiredChassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Drives the robot at a desired chassis speeds. The coordinate system
     * is the same as the one as the one for setPoseMeters().
     * 
     * @param desiredChassisSpeeds - Field relative chassis speeds, in m/s and rad/s. 
     * @param closedLoop - Whether or not to drive the drive wheels with using feedback control.
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds) {
        Rotation2d currentOrientation = odometry.getPoseMeters().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds);
    }

    public ChassisSpeeds getFieldOrientedVelocity() {
        ChassisSpeeds robotOrientedSpeeds = DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotOrientedSpeeds, odometry.getPoseMeters().getRotation());
    }

    public ChassisSpeeds getRobotRelativeVelocityMPS() {
        return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public double getSpeedMetersPerSecond() {
        ChassisSpeeds v = getFieldOrientedVelocity();
        double s = Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
        return s;
    }

    // path following
    public void followTrajectory(Trajectory<SwerveSample> trajectory, Timer timer, boolean isRedAlliance) {
        SwerveSample finalSample= trajectory.getFinalSample(isRedAlliance).get();
        SwerveSample sample = finalSample.t <= timer.get() ? finalSample : trajectory.sampleAt(timer.get(), isRedAlliance).get();

        // Get the current pose of the robot
        Pose2d pose = odometry.getPoseMeters();
         Logger.recordOutput("choreo target pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));

        // Generate the next velocities for the robot
        ChassisSpeeds velocities = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated velocities
        fieldOrientedDrive(velocities);
    }

    // checkpoint following 
    public void followCheckpointsTrajectory(Trajectory<SwerveSample> trajectory, Timer timer, boolean isRedAlliance, double checkpointToleranceMeters) {
        SwerveSample finalSample= trajectory.getFinalSample(isRedAlliance).get();
        SwerveSample sample = finalSample.t <= timer.get() ? finalSample : trajectory.sampleAt(timer.get(), isRedAlliance).get();

        // Get the current pose of the robot
        Pose2d currentPose = odometry.getPoseMeters();
        Pose2d choreoTargetPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
        Logger.recordOutput("choreo target pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));

        // initializes the target velocities chassis speeds that we input into field oriented drive
        ChassisSpeeds velocities;

        // checks if robot error is too much and if so just use pid to get back to last checkpoint/sample pose
        // and also use 50% of feedforward velocity
        if(checkpointToleranceMeters < Math.abs(choreoTargetPose.minus(currentPose).getTranslation().getNorm())) {

            // stop running the timer that is used for setpoints if timer is running
            if(timer.isRunning()) {
                timer.stop();
            }

            // use 50% of feed forward to less fight the pid if the pose is really off
            velocities = new ChassisSpeeds(
                sample.vx*0.5 + xController.calculate(currentPose.getX(), sample.x),
                sample.vy*0.5 + yController.calculate(currentPose.getY(), sample.y),
                sample.omega*0.5 + headingController.calculate(currentPose.getRotation().getRadians(), sample.heading)
            );
        } else {

            // if tiemer is not running start it up again
            if(!timer.isRunning()) {
                timer.start();
            }

            // Generate the next velocities for the robot
            velocities = new ChassisSpeeds(
                sample.vx + xController.calculate(currentPose.getX(), sample.x),
                sample.vy + yController.calculate(currentPose.getY(), sample.y),
                sample.omega + headingController.calculate(currentPose.getRotation().getRadians(), sample.heading)
            );
        }
    
        // Apply the generated velocities
        fieldOrientedDrive(velocities);
    }

    public boolean isAtEndOfTrajectory(double toleranceMeters, double velocityToleranceMPS, double timeSec, SwerveSample finalSample) {
        Pose2d pose = odometry.getPoseMeters();

        // if the current time is less than 50% of the time it should take to do the path assume path is not done
        if((timeSec / finalSample.t) < 0.5) return false;

        // see if the linear distance between current robot pose and final path pose is within tolerance
        if(pose.getTranslation().getDistance(new Translation2d(finalSample.x, finalSample.y)) <= toleranceMeters) {
            // gets the velocity delta from chassis speeds class 
            ChassisSpeeds velocityDelta = getFieldOrientedVelocity().minus(finalSample.getChassisSpeeds());
            double vxError = Math.abs(velocityDelta.vxMetersPerSecond);
            double vyError = Math.abs(velocityDelta.vyMetersPerSecond);

            // checks if vx and vy are in tolerance
            if((vxError <= velocityToleranceMPS) && (vyError <= velocityToleranceMPS)) {
                return true;
            }

        }
        // if it got to this point then the path isn't done
        return false;
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : swerveModules)
            mod.periodic();
        
        odometry.gyroIO.updateInputs(odometry.gyroInputs);

        Logger.processInputs("gyroInputs", odometry.gyroInputs);

        odometry.updatePoseEstimator();
        

        Logger.recordOutput("drivetrain/fusedPose", odometry.fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", odometry.wheelsOnlyPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());

        Logger.recordOutput("drivetrain/swerveModuleStates", getModuleStates());
        Logger.recordOutput("drivetrain/swerveModulePositions", getModulePositions());

        // this.compareCamPoses();
    }

    @Override
    public void simulationPeriodic() {
        // Move the simulation forward by 1 timestep (just camera stuff for now)
        FieldConstants.simulatedTagLayout.update(odometry.wheelsOnlyPoseEstimator.getEstimatedPosition());
        FieldConstants.simulatedFuelLayout.update(odometry.wheelsOnlyPoseEstimator.getEstimatedPosition());

        ArrayList<Translation3d> simulatedFuel = new ArrayList<>();
        for (VisionTargetSim fuel : FieldConstants.simulatedFuelLayout.getVisionTargets()) {
            simulatedFuel.add(fuel.getPose().getTranslation());
        }

        Logger.recordOutput("simulatedFuel", simulatedFuel.toArray(new Translation3d[0]));
    }
}
