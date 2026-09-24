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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    private void setShot(double velocity, double position) {
       

        io.setShooterVelocity(velocity);
        io.setPivotAngle(position);    
    }

    public double getCurrentShooterRPM() {
        return inputs.shooterVelocity;
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

    public Command setShooterVelocityCommand(double targetRPM) {
        return this.run(() -> io.setShooterVelocity(targetRPM));
    }

    public Command setPivotAngleCommand(double targetDegrees) {
        return this.run(() -> io.setPivotAngle(targetDegrees));
    }

}
