package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class ShootIntoHubCommand extends Command {

    public final Drivetrain drivetrain;
    public final Shooter shooter;
    private final PIDController shooterDrivetrainPid = new PIDController(Constants.ShooterConstants.shootDrivetrainKp,Constants.ShooterConstants.shootDrivetrainKi,Constants.ShooterConstants.shootDrivetrainKd);
    private final SlewRateLimiter distanceLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
    public ShootIntoHubCommand(Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;

        addRequirements(drivetrain,shooter);
    }

    @Override
    public void execute() {
        // use lookup table to get correct shot velocity and hood angle
        Translation2d robotPosition = new Translation2d(drivetrain.odometry.getPoseMeters().getMeasureX(),drivetrain.odometry.getPoseMeters().getMeasureY());
        double distanceToHubMeters = distanceLimiter.calculate(robotPosition.minus(FieldConstants.midField).getNorm());
        Double shotVelocity = Constants.ShooterConstants.velocityMap.get(distanceToHubMeters);
        Double shotAngle = Constants.ShooterConstants.angleMap.get(distanceToHubMeters);

        // trig to get desired angle to hub
        Translation2d robotToHub = FieldConstants.midField.minus(robotPosition);
        double desiredAngleToHub = Math.atan2(robotToHub.getY(), robotToHub.getX());
        double currentAngle = drivetrain.odometry.getPoseMeters().getRotation().getRadians();

        double angularVelocity = MathUtil.clamp(
            shooterDrivetrainPid.calculate(currentAngle, desiredAngleToHub),
            -Constants.DrivetrainConstants.maxAchievableAngularVelocityRadiansPerSecond,
            Constants.DrivetrainConstants.maxAchievableAngularVelocityRadiansPerSecond
        );

        drivetrain.fieldOrientedDrive(new ChassisSpeeds(drivetrain.getRobotRelativeVelocityMPS().vxMetersPerSecond,
                drivetrain.getRobotRelativeVelocityMPS().vyMetersPerSecond, angularVelocity));
        shooter.setShot(shotVelocity, shotAngle);
    }
}
