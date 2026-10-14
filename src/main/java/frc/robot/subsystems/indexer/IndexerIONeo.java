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

    private SparkMaxConfig indexerConfig;
    private SparkMaxConfig indexerFollowConfig;

    public IndexerIONeo() {
        indexerFL = new Neo(Constants.IndexerConstants.indexerFLId);
        indexerFR = new Neo(Constants.IndexerConstants.indexerFRId);
        indexerBL = new Neo(Constants.IndexerConstants.indexerBLId);
        indexerBR = new Neo(Constants.IndexerConstants.indexerBRId);

        configureMotors();
    }

    private void configureMotors() {

        // base config
        indexerConfig = new SparkMaxConfig();
        indexerConfig.smartCurrentLimit(50);
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerConfig.inverted(false); //TODO set real invertion
        indexerConfig.encoder.velocityConversionFactor(1.0/60.0); // convert rpm to rps
        indexerFL.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // indexerConfig.inverted(true);
         indexerBL.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerConfig.inverted(true);
        indexerFR.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerBR.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void setIndexerVolts(double volts) {
        // indexerFL.setVoltage(volts);
        // indexerBL.setVoltage(volts);
        indexerFR.setVoltage(volts);
        indexerBR.setVoltage(volts);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVelocity = indexerFL.getVelocity();

        inputs.indexerVolts = indexerFL.getAppliedOutput() * indexerFL.getBusVoltage();

        inputs.indexerAmps = indexerFL.getOutputCurrent();
    }

}
