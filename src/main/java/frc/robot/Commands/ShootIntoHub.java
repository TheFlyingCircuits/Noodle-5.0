import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class ShootIntoHub extends Commands{

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final SimpleMotorFeedforward feedforward= new SimpleMotorFeedforward(Constants.ShooterConstants.shootDrivetrainHKs,Constants.ShooterConstants.shootDrivetrainHKd);
    private final PIDController shooterPid = new PIDController(Constants.ShooterConstants.shootDrivetrainHKp,Constants.ShooterConstants.shootDrivetrainHKi,Constants.ShooterConstants.shootDrivetrainHKd);
    private final SlewRateLimiter distanceLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
    public ShootIntoHub(Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;

    }

    @Override
    public void execute() {
        double distanceToHubMeters = distanceLimiter.calculate(new Translation2d(drivetrain.odometry.getPoseMeters().getMeasureX(),drivetrain.odometry.getPoseMeters().getMeasureY() ).minus(FieldConstants.midField).getNorm());
        Double shotVelocity = Constants.ShooterConstants.velocityMap.get(distanceToHubMeters);
        Double shotAngle = Constants.ShooterConstants.angleMap.get(distanceToHubMeters);
        
        
    }
}
