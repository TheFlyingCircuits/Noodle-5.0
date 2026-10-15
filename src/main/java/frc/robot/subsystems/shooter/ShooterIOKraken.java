package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Kraken;

public class ShooterIOKraken implements ShooterIO {

    private Kraken shooterTL;
    private Kraken shooterTR;
    private Kraken shooterBL;
    private Kraken shooterBR;
    private Kraken pivot;

    private TalonFXConfiguration drumConfig;
    private TalonFXConfiguration pivotConfig;

    public ShooterIOKraken() {
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
        drumConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        drumConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.statorCurrentLimitAmps;
        drumConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ShooterConstants.statorCurrentLimitAmps;
        // drumConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.ShooterConstants.statorCurrentLimitAmps;
        drumConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;
        drumConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        drumConfig.Slot0.kS = Constants.ShooterConstants.shooterVelocityKs;
        drumConfig.Slot0.kV = Constants.ShooterConstants.shooterVelocityKv;
        drumConfig.Slot0.kP = Constants.ShooterConstants.shooterVelocityKp;

        drumConfig.Feedback.SensorToMechanismRatio = 40.0/18.0;

        shooterTL.applyConfig(drumConfig);
        shooterBL.applyConfig(drumConfig);

        drumConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterTR.applyConfig(drumConfig);
        shooterBR.applyConfig(drumConfig);

        // shooterTR.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Opposed));
        // shooterBL.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Aligned));
        // shooterBR.setControl(new Follower(shooterTL.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configurePivotMotor() {
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.pivotCurrentLimitAmps;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotConfig.Slot0.kS = Constants.ShooterConstants.pivotKs;
        pivotConfig.Slot0.kP = Constants.ShooterConstants.pivotKp;

        // TODO: this is fake gear ratio because it will prob not be 1:1
        pivotConfig.Feedback.SensorToMechanismRatio = (22.0/18.0) * (146.0/10.0);

        pivot.applyConfig(pivotConfig);
        pivot.setPosition(Units.degreesToRotations(11));
    }

    @Override
    public void setShooterVolts(double volts) {
        shooterTL.setControl(new VoltageOut(volts));
        shooterBL.setControl(new VoltageOut(volts));
        shooterTR.setControl(new VoltageOut(volts));
        shooterBR.setControl(new VoltageOut(volts));
    }

    @Override
    public void setShooterCurrent(double amps) {
        shooterTL.setControl(new TorqueCurrentFOC(Amps.of(amps)));
        shooterBL.setControl(new TorqueCurrentFOC(Amps.of(amps)));
        shooterTR.setControl(new TorqueCurrentFOC(Amps.of(amps)));
        shooterBR.setControl(new TorqueCurrentFOC(Amps.of(amps)));
    }

    @Override
    public void setShooterVelocity(double shooterVelocityRPS) {
        shooterTL.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(shooterVelocityRPS)));
        shooterBL.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(shooterVelocityRPS)));
        shooterTR.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(shooterVelocityRPS)));
        shooterBR.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(shooterVelocityRPS)));
    }

    @Override
    public void setPivotVolts(double volts) {
        pivot.setControl(new VoltageOut(volts));
    }

    @Override
    public void setPivotAngle(double targetAngleDegrees) {
        targetAngleDegrees = MathUtil.clamp(targetAngleDegrees, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
        pivot.setControl(new PositionVoltage(Units.degreesToRotations(targetAngleDegrees)));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
    
        inputs.shooterVelocityRPS = shooterTL.getVelocity().getValueAsDouble();


        inputs.shooterVolts = shooterTL.getMotorVoltage().getValueAsDouble();
        inputs.pivotVolts = pivot.getMotorVoltage().getValueAsDouble();

        inputs.shooterAmps = shooterTL.getStatorCurrent().getValueAsDouble();
        inputs.pivotAmps = pivot.getStatorCurrent().getValueAsDouble();

        inputs.pivotAngleDegrees = Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble());
    }
    
}
