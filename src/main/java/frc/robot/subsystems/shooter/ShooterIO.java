package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public class ShooterIOInputs {
        public double shooterTLVelocity = 0.0;
        public double shooterTRVelocity = 0.0;
        public double shooterBLVelocity = 0.0;
        public double shooterBRVelocity = 0.0;

        public double shooterTLVolts = 0.0;
        public double shooterTRVolts = 0.0;
        public double shooterBLVolts = 0.0;
        public double shooterBRVolts = 0.0;
        public double pivotVolts = 0.0;

        public double shooterTLAmps = 0.0;
        public double shooterTRAmps = 0.0;
        public double shooterBLAmps = 0.0;
        public double shooterBRAmps = 0.0;
        public double pivotAmps = 0.0;

        public double pivotAngleDegrees = 0.0;

    }
    
    public default void updateInputs(ShooterIOInputs inputs) {
    }
    
    public default void setShooterVolts(double volts) {
    }

    public default void setShooterVelocity(double targetRPM) {
    }

    public default void setPivotVolts(double volts) {
    }

    public default void setPivotAngle(double targetAngleDegrees) {
    }
    
}
