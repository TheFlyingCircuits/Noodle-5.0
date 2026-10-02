package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterError;

public class PassCommand extends Command {

    private enum PossibeTargets {
        PASSING_LEFT,
        PASSING_RIGHT,
        PASSING_MID_RIGHT,
        PASSING_MID_LEFT
    }

    /**
     * A record to keep track of information about shooting fuel and its trajectory
     *
     * @param exitVelocityMPS The exit velocity of the fuel when it is shot out (meters per second)
     * @param exitAngleDeg The exit angle of the fuel when it is shot out (degrees)
     * @param tImpactDelta The time delta from when fuel is first shot to when it hits the target (seconds)
     */
    public record FuelTrajectory(double exitVelocityMPS, double exitAngleDeg, double tImpactDelta) {}


    private PossibeTargets shootingTarget;
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;

    private final Transform3d shooterTranslationRobotRelative = new Transform3d(-Units.inchesToMeters(13.5), 0.0, Units.inchesToMeters(21), new Rotation3d());

    // gravity constant
    private final double g = 9.81;
    private final double sideWaysToleranceMeters = 0.75;
    private boolean isShooting = false;

    private double driveErrorDeg = 9999.0;

    public PassCommand(Drivetrain drivetrain, Shooter shooter, Indexer indexer, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer=indexer;
        this.intake=intake;

        addRequirements(drivetrain,shooter);
    }

    @Override 
    public void initialize() {
        shootingTarget = PossibeTargets.PASSING_LEFT;
        driveErrorDeg = 9999.0;
        isShooting = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.odometry.getPoseMeters();
        Translation2d robotTranslation = robotPose.getTranslation();

        updateShootingTarget(robotTranslation);
        Translation3d passingTargetTranslation = getPassingTargetTranslation(shootingTarget);


        Translation3d shooterTranslationFieldRelative = new Pose3d(robotPose).plus(shooterTranslationRobotRelative).getTranslation();
        Logger.recordOutput("Shooter translation" ,shooterTranslationFieldRelative);

        FuelTrajectory targetFuelTrajectory = angleOfAttackTrajCalc(passingTargetTranslation, 45.0, shooterTranslationFieldRelative);

        // convert meters per second of fuel output to rps of our drum, 60 sec *(m/s / pi*diamter)
        final double targetVelocityRPS = 60.0 * (targetFuelTrajectory.exitVelocityMPS / (Math.PI*ShooterConstants.drumDiameterMeters));

        // gets the shooter to spin a target RPS and the pivot to aim at target angle, this also returns the shooter error record.
        final ShooterError shooterError = shooter.setShot(targetVelocityRPS, targetFuelTrajectory.exitAngleDeg);
        Logger.recordOutput("Passing target", passingTargetTranslation);

        if(!isShooting) {
            // gets the drivetrain to aim at passing target and it retuns the error in radians in wich we convert to degrees
            driveErrorDeg = Units.radiansToDegrees(drivetrain.aimAtTranslation(passingTargetTranslation.toTranslation2d(), true));
        } else {
            drivetrain.swerveXPattern();
        }

        boolean drivetrainInTolerance = UniversalConstants.drivetrainShotToleranceDeg > Math.abs(driveErrorDeg);
        boolean shotVelocityInTolerance = UniversalConstants.shooterShotRPMTolerance > Math.abs(shooterError.velocityErrorRPM());
        boolean shotAngleInTolerance = UniversalConstants.shooterShotTolgeranceDeg > Math.abs(shooterError.angleErrorDeg());

        // if not shooting check if everything is within tolerance and if so then change isShooing to true and start shooting
        if(!isShooting) isShooting = ((drivetrainInTolerance && shotVelocityInTolerance && shotAngleInTolerance));

        if(isShooting) {
            indexer.setIndexerVelocity(2000.0);
            intake.intakeUpAndIntake();
        } else {
            indexer.stopIndexing();
            intake.intakeDefault();
        }

        // logs the errors
        Logger.recordOutput("Passing/Drivetrain errorDeg", driveErrorDeg);
        Logger.recordOutput("Passing/Shooter velocityErrorRPM", shooterError.velocityErrorRPM());
        Logger.recordOutput("Passing/Shooter angleErrorDeg", shooterError.angleErrorDeg());
        Logger.recordOutput("Passing/isShooting", isShooting);
    }

    private FuelTrajectory angleOfAttackTrajCalc(Translation3d targetTranslation, double angleOfAttackDegrees, Translation3d shooterTranslation) {
        Translation3d targetTranslationTurretRelative = targetTranslation.minus(shooterTranslation);

        double ballDisplacementXYMeters = Math.sqrt(Math.pow(targetTranslationTurretRelative.getX(), 2)
            + Math.pow(targetTranslationTurretRelative.getY(),2)); // same as math.hypot but I wanted to write it out one time

        // ballDisplacementXYMeters = targetTranslationTurretRelative.toTranslation2d().getNorm(); is easy way

        double ballDisplacementZMeters = targetTranslationTurretRelative.getZ();

        double timeOfImpactSeconds = Math.sqrt(Math.abs((2.0/g)*(ballDisplacementZMeters
            -ballDisplacementXYMeters*Math.tan(Units.degreesToRadians(angleOfAttackDegrees)))));

        double initialVelocityXYMetersPerSecond = ballDisplacementXYMeters/timeOfImpactSeconds;
        double initialVelocityZMetersPerSecond = (ballDisplacementZMeters+(0.5*g)*Math.pow(timeOfImpactSeconds,2))/timeOfImpactSeconds;
        double exitVelocityMetersPerSecond = Math.hypot(initialVelocityXYMetersPerSecond, initialVelocityZMetersPerSecond);

        double launchAngleRadians = Math.atan2(initialVelocityZMetersPerSecond, initialVelocityXYMetersPerSecond);
        double launchAngleDegrees = Units.radiansToDegrees(launchAngleRadians);

        return new FuelTrajectory(exitVelocityMetersPerSecond, launchAngleDegrees, timeOfImpactSeconds);
    }

    public static Translation3d getPassingTargetTranslation(PossibeTargets passingTarget) {
        
        double inverIfRed = FlyingCircuitUtils.getAllianceDependentValue(-1.0, 1.0, 1.0);
        Translation3d target;

        if(passingTarget == PossibeTargets.PASSING_LEFT) {
            // new Translation3d(0.500, 6.448,0.0); for test
            target=FieldElement.TRENCH_LEFT.getLocation().plus(new Translation3d(-1.5*inverIfRed,-1.5*inverIfRed,-0.55));
        } else if (passingTarget == PossibeTargets.PASSING_RIGHT){
            target=FieldElement.TRENCH_RIGHT.getLocation().plus(new Translation3d(-1.5*inverIfRed,1.5*inverIfRed,-0.55));
        } else if (passingTarget == PossibeTargets.PASSING_MID_LEFT){
            target=FieldElement.TRENCH_LEFT.getLocation().plus(new Translation3d(3.5*inverIfRed,-1.5*inverIfRed,-0.55));
        }else {
            target=FieldElement.TRENCH_RIGHT.getLocation().plus(new Translation3d(3.5*inverIfRed,1.5*inverIfRed,-0.55));
        }

        return target;
    }

    private void updateShootingTarget(Translation2d robotTranslation) {

        double distanceToLeftTrench = FieldElement.TRENCH_LEFT.getLocation2d().getDistance(robotTranslation);
        double distanceToRightTrench = FieldElement.TRENCH_RIGHT.getLocation2d().getDistance(robotTranslation);

        try {
            if(shootingTarget == PossibeTargets.PASSING_LEFT || shootingTarget == PossibeTargets.PASSING_MID_LEFT) {
                if(!(shootingTarget == PossibeTargets.PASSING_MID_LEFT)) {
                    distanceToLeftTrench = distanceToLeftTrench - sideWaysToleranceMeters;
                }
                distanceToLeftTrench = distanceToLeftTrench - sideWaysToleranceMeters;
            } else {
                if(!(shootingTarget == PossibeTargets.PASSING_MID_RIGHT)) {
                    distanceToRightTrench = distanceToRightTrench - sideWaysToleranceMeters;
                }
            }
            
            
        } catch(NullPointerException e) {}


        boolean isPastRedHub = robotTranslation.getX() > FieldElement.Enemy_HUB.getLocation().getX();
        boolean isPastBlueHub = !isPastRedHub;
        boolean shouldMidPass = FlyingCircuitUtils.getAllianceDependentValue(isPastBlueHub, isPastRedHub, true);
        if(distanceToLeftTrench<distanceToRightTrench) {
            if(!(shouldMidPass)) {
                shootingTarget = PossibeTargets.PASSING_LEFT;
            } else {
                shootingTarget = PossibeTargets.PASSING_MID_LEFT;
            }
        } else {
            if(!(shouldMidPass)) {
                shootingTarget = PossibeTargets.PASSING_RIGHT;
            } else {
                shootingTarget = PossibeTargets.PASSING_MID_RIGHT;
            }
        }
    }
    
}
