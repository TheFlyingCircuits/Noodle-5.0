package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.IntakeConstants;

public interface IntakeIO {
   @AutoLog
   public class IntakeIOInputs {
      public double intakePositionDegrees = IntakeConstants.intakeStartDegrees;

      public double intakeVelocityRPS = 0.0;
      public double rollerTopVelocityRPS = 0.0;
      public double rollerBottomVelocityRPS = 0.0;

      public double intakeVelocityDegreesPerSecond = 0.0;

      public double intakeVolts = 0.0;
      public double rollerTopVolts = 0.0;
      public double rollerBottomVolts = 0.0;

      public double intakeAmps = 0.0;
      public double rollerTopAmps = 0.0;
      public double rollerBottomAmps = 0.0;
   }
  
   public default void updateInputs(IntakeIOInputs inputs) {};
      
   public default void setRollerTopVolts(double volts) {};

   public default void setRollerBottomVolts(double volts) {};

   public default void setIntakeVolts(double volts) {};

   public default void setTargetRollerTopVelocity(double velocityRPS) {};

   public default void setTargetRollerBottomVelocity(double velocityRPS) {};

   public default void setTargetIntakePositionDegrees(double degrees) {};
}
