package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {

    double simShooterRPM = 0.0;
    double simPivotDegrees = 0.0;


    public ShooterIOSim() {}

    @Override
    public void setShooterVolts(double volts) {}

    @Override
    public void setShooterVelocity(double targetRPM) {
        simShooterRPM = targetRPM;
    }

    @Override
    public void setPivotVolts(double volts) {}

    @Override
    public void setPivotAngle(double targetAngleDegrees) {
        simPivotDegrees = targetAngleDegrees;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // System.out.println(simShooterRPM);
        inputs.shooterVelocityRPM = simShooterRPM;


        // inputs.shooterVolts = shooterTL.getMotorVoltage().getValueAsDouble();
        // inputs.pivotVolts = pivot.getMotorVoltage().getValueAsDouble();

        // inputs.shooterAmps = shooterTL.getStatorCurrent().getValueAsDouble();
        // inputs.pivotAmps = pivot.getStatorCurrent().getValueAsDouble();

        inputs.pivotAngleDegrees = simPivotDegrees;
    }
    
}
