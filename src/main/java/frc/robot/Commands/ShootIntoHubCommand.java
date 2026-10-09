package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterError;

public class ShootIntoHubCommand extends Command {

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;


    private boolean isShooting = false;
    private Double driveErrorDeg = 9999.0;
    private ShooterError shooterError = new ShooterError(9999.0, 9999.0);

    private InterpolatingDoubleTreeMap velocityMap = Constants.ShooterConstants.velocityMap;
    private InterpolatingDoubleTreeMap angleMap = Constants.ShooterConstants.angleMap;



    public ShootIntoHubCommand(Drivetrain drivetrain, Shooter shooter, Indexer indexer, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(drivetrain, shooter,indexer,intake);
    }

    @Override
    public void initialize() {
        drivetrain.odometry.allowTeleportsNextPoseUpdate();
        drivetrain.odometry.fullyTrustVisionNextPoseUpdate();
        isShooting = false;
        driveErrorDeg = 9999.0;
    }

    @Override
    public void execute() {
        Double robotDistanceToHub = drivetrain.odometry.getPoseMeters().getTranslation().getDistance(FieldElement.HUB.getPose2d().getTranslation());

        boolean drivetrainInTolerance = UniversalConstants.drivetrainShotToleranceDeg > Math.abs(driveErrorDeg);
        boolean shotVelocityInTolerance = UniversalConstants.shooterShotRPSTolerance > Math.abs(shooterError.velocityErrorRPS());
        boolean shotAngleInTolerance = UniversalConstants.shooterShotTolgeranceDeg > Math.abs(shooterError.angleErrorDeg());

        // once is shooting is on it does not go off
        if (!isShooting) isShooting = drivetrainInTolerance && shotVelocityInTolerance && shotAngleInTolerance;

        if (isShooting) { // when shooting put dt in x to stop moving, run intake and indexer
            drivetrain.swerveXPattern();
            indexer.setIndexerVelocity(IndexerConstants.shootingIndexingVelocityRPS);
            intake.intakeUpAndIntake();
        } else { // when not shooting try to get to desired angle and shot speeds
            driveErrorDeg = Units.radiansToDegrees(drivetrain.aimAtTranslation(FieldElement.HUB.getPose2d().getTranslation(), true));
            shooterError = shooter.setShot(velocityMap.get(robotDistanceToHub), angleMap.get(robotDistanceToHub));
            indexer.stopIndexing();
            intake.intakeDefault();
            
        }

        Logger.recordOutput("ShootIntoHubCommand/driveErrorDeg", driveErrorDeg);
        Logger.recordOutput("ShootIntoHubCommand/isShooting", isShooting);
        Logger.recordOutput("ShootIntoHubCommand/shotVelocityErrorRPS", shooterError.velocityErrorRPS());
        Logger.recordOutput("ShootIntoHubCommand/shotAngleError", shooterError.angleErrorDeg());

    }
}
