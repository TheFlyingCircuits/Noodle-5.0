package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.VendorWrappers.Neo;

public class IndexerIONeo implements IndexerIO {
    
    private Neo indexerFL;
    private Neo indexerFR;
    private Neo indexerBL;
    private Neo indexerBR;
    private CANrange indexerRange;

    private SparkMaxConfig indexerConfig;
    private SparkMaxConfig indexerFollowConfig;

    public IndexerIONeo() {
        indexerFL = new Neo(Constants.IndexerConstants.indexerFLId);
        indexerFR = new Neo(Constants.IndexerConstants.indexerFRId);
        indexerBL = new Neo(Constants.IndexerConstants.indexerBLId);
        indexerBR = new Neo(Constants.IndexerConstants.indexerBRId);
        indexerRange = new CANrange(
            Constants.IndexerConstants.indexerRangeId,
            Constants.UniversalConstants.canivoreName);

        configureMotors();
    }

    private void configureMotors() {

        // base config
        indexerConfig = new SparkMaxConfig();
        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false); //TODO set real invertion
        indexerConfig.encoder.positionConversionFactor(1); //TODO set real conversion factor
        indexerFL.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // follow config
        indexerFollowConfig = indexerConfig;
        indexerFollowConfig.follow(Constants.IndexerConstants.indexerFLId);

        indexerFR.configure(indexerFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerBL.configure(indexerFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerBR.configure(indexerFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void setIndexerVolts(double volts) {
        indexerFL.setVoltage(volts);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerFLVelocity = indexerFL.getVelocity();
        inputs.indexerFRVelocity = indexerFR.getVelocity();
        inputs.indexerBLVelocity = indexerBL.getVelocity();
        inputs.indexerBRVelocity = indexerBR.getVelocity();

        inputs.indexerFLVolts = indexerFL.getAppliedOutput() * indexerFL.getBusVoltage();
        inputs.indexerFRVolts = indexerFR.getAppliedOutput() * indexerFR.getBusVoltage();
        inputs.indexerBLVolts = indexerBL.getAppliedOutput() * indexerBL.getBusVoltage();
        inputs.indexerBRVolts = indexerBR.getAppliedOutput() * indexerBR.getBusVoltage();

        inputs.indexerFLAmps = indexerFL.getOutputCurrent();
        inputs.indexerFRAmps = indexerFR.getOutputCurrent();
        inputs.indexerBLAmps = indexerBL.getOutputCurrent();
        inputs.indexerBRAmps = indexerBR.getOutputCurrent();

        inputs.indexerRangeDistanceMeters = indexerRange.getDistance().getValueAsDouble();
    }

}
