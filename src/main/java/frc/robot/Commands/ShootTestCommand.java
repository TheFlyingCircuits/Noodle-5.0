package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterError;

public class ShootTestCommand extends Command {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Intake intake;


    private boolean isShooting = false;
    private ShooterError shooterError;

    private double desiredHoodAngleDeg; 
    private double desiredVelocityRPS;



    public ShootTestCommand( Shooter shooter, Indexer indexer, Intake intake) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        shooterError = new ShooterError(9999.0, 9999.0);

        addRequirements(shooter,indexer,intake);
    }

    @Override
    public void initialize() {
        isShooting = false;
        desiredHoodAngleDeg = 30.0;
        desiredVelocityRPS = 20.0;
        shooterError = new ShooterError(9999.0, 9999.0);
    }

    @Override
    public void execute() {

        shooterError = shooter.setShot(desiredVelocityRPS, desiredHoodAngleDeg);

        boolean shotVelocityInTolerance = UniversalConstants.shooterShotRPSTolerance > Math.abs(shooterError.velocityErrorRPS());
        boolean shotAngleInTolerance = UniversalConstants.shooterShotTolgeranceDeg > Math.abs(shooterError.angleErrorDeg());

        // once is shooting is on it does not go off

        if (!isShooting) isShooting = shotVelocityInTolerance && shotAngleInTolerance;

        if (isShooting) { // when shooting put dt in x to stop moving, run intake and indexer
            indexer.setIndexerVelocity(IndexerConstants.shootingIndexingVelocityRPS);
            intake.intakeUpAndIntake();
        } else { // when not shooting try to get to desired angle and shot speeds
            indexer.stopIndexing();
            intake.intakeDefault();
            
        }

        Logger.recordOutput("ShootIntoHubCommand/isShooting", isShooting);
        Logger.recordOutput("ShootIntoHubCommand/shotVelocityErrorRPS", shooterError.velocityErrorRPS());
        Logger.recordOutput("ShootIntoHubCommand/shotAngleError", shooterError.angleErrorDeg());

    }
}
