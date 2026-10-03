package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterError;

public class ShootIntoHubCommand extends Command {

    public final Drivetrain drivetrain;
    public final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;


    private boolean isShooting = false;
    private double driveErrorDeg = 9999.0;
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
        driveErrorDeg = drivetrain.aimAtTranslation(FieldConstants.midField, false);
        shooterError = shooter.setShot(velocityMap.get(drivetrain.odometry.getPoseMeters().getTranslation().getDistance(FieldElement.HUB.getPose2d().getTranslation())), angleMap.get(drivetrain.odometry.getPoseMeters().getTranslation().getDistance(FieldElement.HUB.getPose2d().getTranslation())));

        boolean drivetrainInTolerance = UniversalConstants.drivetrainShotToleranceDeg > Math.abs(driveErrorDeg);
        boolean shotVelocityInTolerance = UniversalConstants.shooterShotRPMTolerance > Math.abs(shooterError.velocityErrorRPM());
        boolean shotAngleInTolerance = UniversalConstants.shooterShotTolgeranceDeg > Math.abs(shooterError.angleErrorDeg());

       if (drivetrainInTolerance && shotVelocityInTolerance && shotAngleInTolerance) {
            isShooting = true;
            indexer.setIndexerVelocity(2000.0);
            intake.intakeUpAndIntake();
        } else {
            isShooting = false;
            indexer.stopIndexing();
            intake.intakeDefault();
        }

        Logger.recordOutput("ShootIntoHubCommand/driveErrorDeg", driveErrorDeg);
        Logger.recordOutput("ShootIntoHubCommand/isShooting", isShooting);
        Logger.recordOutput("ShootIntoHubCommand/shotVelocityError", shooterError.velocityErrorRPM());
        Logger.recordOutput("ShootIntoHubCommand/shotAngleError", shooterError.angleErrorDeg());

    }
}
