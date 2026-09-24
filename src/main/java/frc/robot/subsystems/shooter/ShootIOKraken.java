package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.VendorWrappers.Kraken;

public class ShootIOKraken implements ShooterIO {

    private Kraken shooterTL;
    private Kraken shooterTR;
    private Kraken shooterBL;
    private Kraken shooterBR;
    private Kraken pivot;

    private TalonFXConfiguration drumConfig;
    private TalonFXConfiguration pivotConfig;

    public ShootIOKraken() {

        drumConfig = new TalonFXConfiguration();
        pivotConfig = new TalonFXConfiguration();
        shooterTL = new Kraken("Shooter TL", Constants.ShooterConstants.shooterTLId, Constants.UniversalConstants.canivoreName);
        shooterTR = new Kraken("Shooter TR", Constants.ShooterConstants.shooterTRId, Constants.UniversalConstants.canivoreName);
        shooterBL = new Kraken("Shooter BL", Constants.ShooterConstants.shooterBLId, Constants.UniversalConstants.canivoreName);
        shooterBR = new Kraken("Shooter BR", Constants.ShooterConstants.shooterBRId, Constants.UniversalConstants.canivoreName);
        pivot = new Kraken("Shooter Pivot", Constants.ShooterConstants.pivotId, Constants.UniversalConstants.canivoreName);
        configureDrumMotors();
        configurePivotMotor();

    }

    private void configureDrumMotors() {
        drumConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        drumConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        drumConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.statorCurrentLimitAmps;
        drumConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        drumConfig.Slot0.kS = Constants.ShooterConstants.shooterVelocityKs;
        drumConfig.Slot0.kV = Constants.ShooterConstants.shooterVelocityKv;
        drumConfig.Slot0.kP = Constants.ShooterConstants.shooterVelocityKp;
        drumConfig.Slot0.kI = Constants.ShooterConstants.shooterVelocityKi;
        drumConfig.Slot0.kD = Constants.ShooterConstants.shooterVelocityKd;

        drumConfig.Feedback.SensorToMechanismRatio = 1.0;

        shooterTL.applyConfig(drumConfig);

        shooterTR.applyConfig(drumConfig);
        shooterBL.applyConfig(drumConfig);
        shooterBR.applyConfig(drumConfig);

        shooterTR.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterBL.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterBR.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configurePivotMotor() {
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.pivotCurrentLimitAmps;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotConfig.Slot0.kS = Constants.ShooterConstants.pivotKs;
        pivotConfig.Slot0.kV = Constants.ShooterConstants.pivotKv;
        pivotConfig.Slot0.kP = Constants.ShooterConstants.pivotKp;
        pivotConfig.Slot0.kI = Constants.ShooterConstants.pivotKi;
        pivotConfig.Slot0.kD = Constants.ShooterConstants.pivotKd;

        pivotConfig.Feedback.SensorToMechanismRatio = 1.0;

        pivot.applyConfig(pivotConfig);
    }

    @Override
    public void setShooterVolts(double volts) {
        shooterTL.setControl(new VoltageOut(volts));
    }

    @Override
    public void setShooterVelocity(double targetRPM) {
        shooterTL.setControl(new VelocityVoltage(targetRPM / 60.0));
    }

    @Override
    public void setPivotVolts(double volts) {
        pivot.setControl(new VoltageOut(volts));
    }

    @Override
    public void setPivotAngle(double targetAngleDegrees) {
        pivot.setControl(new PositionVoltage(Units.degreesToRotations(targetAngleDegrees)));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterTLVelocity = shooterTL.getVelocity().getValueAsDouble() * 60.0;
        inputs.shooterTRVelocity = shooterTR.getVelocity().getValueAsDouble() * 60.0;
        inputs.shooterBLVelocity = shooterBL.getVelocity().getValueAsDouble() * 60.0;
        inputs.shooterBRVelocity = shooterBR.getVelocity().getValueAsDouble() * 60.0;

        inputs.shooterTLVolts = shooterTL.getMotorVoltage().getValueAsDouble();
        inputs.shooterTRVolts = shooterTR.getMotorVoltage().getValueAsDouble();
        inputs.shooterBLVolts = shooterBL.getMotorVoltage().getValueAsDouble();
        inputs.shooterBRVolts = shooterBR.getMotorVoltage().getValueAsDouble();
        inputs.pivotVolts = pivot.getMotorVoltage().getValueAsDouble();

        inputs.shooterTLAmps = shooterTL.getStatorCurrent().getValueAsDouble();
        inputs.shooterTRAmps = shooterTR.getStatorCurrent().getValueAsDouble();
        inputs.shooterBLAmps = shooterBL.getStatorCurrent().getValueAsDouble();
        inputs.shooterBRAmps = shooterBR.getStatorCurrent().getValueAsDouble();
        inputs.pivotAmps = pivot.getStatorCurrent().getValueAsDouble();

        inputs.pivotAngleDegrees = Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble());
    }
    
}
