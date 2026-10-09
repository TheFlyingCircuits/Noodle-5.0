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

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * This record is used to store the error for the shooter components.
     * @param velocityErrorRPS The error of the drum shooter velocity in RPS from target-current.
     * @param angleErrorDeg The error of the pivot in degrees from target-current.
     */
    public record ShooterError(double velocityErrorRPS, double angleErrorDeg) {};

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * This function is used for setting the target velocity and position for the shooter subsystem.
     * @param velocityRPS Target velocity for the drum shooter and hood shooter in RPS.
     * @param positionDeg Target angle for our pivot/hood in degrees.
     * @return A ShooterError record that has the velocity error in RPS and the angle error in degrees.
     */
    public ShooterError setShot(double velocityRPS, double positionDeg) {
        io.setShooterVelocity(velocityRPS);
        io.setPivotAngle(positionDeg);  

        double errorRPS = velocityRPS - inputs.shooterVelocityRPS;
        double errorDeg = positionDeg - inputs.pivotAngleDegrees;
        
        return new ShooterError(errorRPS, errorDeg);
    }

    public double getCurrentShooterRPS() {
        return inputs.shooterVelocityRPS;
    }

    public double getCurrentPivotAngleDegrees() {
        return inputs.pivotAngleDegrees;
    }

    public Command setShotCommand(double velocity, double position) {
        return this.run(() -> setShot(velocity, position));
    }

    public Command setShooterVoltsCommand(double volts) {
        return this.run(() -> io.setShooterVolts(volts));
    }

    public Command setPivotVoltsCommand(double volts) {
        return this.run(() -> io.setPivotVolts(volts));
    }

    public Command setShooterVelocityCommand(double targetRPS) {
        return this.run(() -> io.setShooterVelocity(targetRPS));
    }

    public Command setPivotAngleCommand(double targetDegrees) {
        return this.run(() -> io.setPivotAngle(targetDegrees));
    }

}
