package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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

    /** error measured in degrees, output is in degrees per second. */
    private PIDController angleController;

    /** error measured in meters, output is in meters per second. */
    private PIDController translationController;
 
    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO
    ) {
        

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0, "frontLeft"),
            new SwerveModule(frSwerveModuleIO, 1, "frontRight"),
            new SwerveModule(blSwerveModuleIO, 2, "backLeft"),
            new SwerveModule(brSwerveModuleIO, 3, "backRight")
        };

        this.odometry = new Odometry(this, gyroIO);
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
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        // previousSetpoint = setpointGenerator.generateSetpoint(
        //     previousSetpoint, // The previous setpoint
        //     desiredChassisSpeeds, // The desired target speeds
        //     0.02 // The loop time of the robot code, in seconds
        // );
        // setModuleStates(previousSetpoint.moduleStates());
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
