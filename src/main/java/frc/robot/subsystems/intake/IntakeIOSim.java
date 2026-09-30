package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class IntakeIOSim implements IntakeIO {

    double deltaT = 0.2;
    double simPosDegrees = 0;
    double simTargetPosDegrees = 90;
    double simVelDegPerSec = 0.0;
    double simTopVelocityRPS = 0.0;
    double simBottomVelocityRPS = 0.0;

    Drivetrain drivetrain;

    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;
    boolean isIntakeDown = false;

    public IntakeIOSim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void setTargetRollerTopVelocity(double velocity) {
        simTopVelocityRPS = velocity;
    };

    @Override
    public void setTargetRollerBottomVelocity(double velocity) {
        simBottomVelocityRPS = velocity;
    };

    @Override
    public void setIntakeVolts(double volts) {
        simVelDegPerSec = volts * 45;
    };

    @Override
    public void setTargetIntakePositionDegrees(double degrees) {
        double simVolts = 0;
        simTargetPosDegrees = degrees;
        simVolts = 0.2 * (simTargetPosDegrees - simPosDegrees);

        setIntakeVolts(simVolts);
    };
    
   @Override
   public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerTopVelocityRPS = simTopVelocityRPS;
        inputs.rollerBottomVelocityRPS = simBottomVelocityRPS;
        inputs.intakePositionDegrees += simVelDegPerSec * deltaT;
        simPosDegrees = inputs.intakePositionDegrees;
        inputs.intakeVelocityDegreesPerSecond = simVelDegPerSec;

        double robotYawRad = drivetrain.odometry.getPoseMeters().getRotation().getRadians();

        Translation2d intakeTranslation2d = drivetrain.odometry.getPoseMeters().getTranslation().plus(new Translation2d(Math.cos(robotYawRad)* 0.1,Math.sin(robotYawRad)*0.0));

        Pose3d intakePoseOnRobotSIM = new Pose3d(new Translation3d(intakeTranslation2d.getX(),intakeTranslation2d.getY(),0.0), 
            new Rotation3d(0,0,drivetrain.odometry.getPoseMeters().getRotation().getRadians()+Units.degreesToRadians(inputs.intakePositionDegrees)));

        Logger.recordOutput("intakePoseOnRobotSIM", intakePoseOnRobotSIM);
    
   }
}