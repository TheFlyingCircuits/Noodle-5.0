package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // used to eliminate teleportation affeting lookup table
    private final SlewRateLimiter distanceLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.maxAchievableVelocityMetersPerSecond);

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    private void setShotForRobotPosition(Translation2d robotPosition) {
        double distanceToHubMeters = distanceLimiter.calculate(robotPosition.minus(FieldConstants.midField).getNorm());
        Double shotVelocity = Constants.ShooterConstants.velocityMap.get(distanceToHubMeters);
        Double shotAngle = Constants.ShooterConstants.angleMap.get(distanceToHubMeters);

        io.setShooterVelocity(shotVelocity);
        io.setPivotAngle(shotAngle);    
    }

    public double getCurrentShooterRPM() {
        return inputs.shooterVelocity;
    }

    public double getCurrentPivotAngleDegrees() {
        return inputs.pivotAngleDegrees;
    }

    public Command shootAtPositionCommand(Translation2d robotPosition) {
        return this.run(() -> setShotForRobotPosition(robotPosition));
    }

    public Command setShooterVoltsCommand(double volts) {
        return this.run(() -> io.setShooterVolts(volts));
    }

    public Command setPivotVoltsCommand(double volts) {
        return this.run(() -> io.setPivotVolts(volts));
    }

    public Command setShooterVelocityCommand(double targetRPM) {
        return this.run(() -> io.setShooterVelocity(targetRPM));
    }

    public Command setPivotAngleCommand(double targetDegrees) {
        return this.run(() -> io.setPivotAngle(targetDegrees));
    }

}
