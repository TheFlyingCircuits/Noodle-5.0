package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public class ShooterIOInputs {
        public double shooterVelocityRPS = 0.0;

        public double shooterVolts = 0.0;
        public double pivotVolts = 0.0;

        public double shooterAmps = 0.0;
        public double pivotAmps = 0.0;

        public double pivotAngleDegrees = 0.0;

    }
    
    public default void updateInputs(ShooterIOInputs inputs) {
    }
    
    public default void setShooterVolts(double volts) {
    }

    public default void setShooterVelocity(double targetRPS) {
    }

    public default void setPivotVolts(double volts) {
    }

    public default void setPivotAngle(double targetAngleDegrees) {
    }
    
}
