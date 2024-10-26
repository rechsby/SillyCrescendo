package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double frontMotorVelocityRotationsPerMinute = 0.0;
    public double rearMotorVelocityRotationsPerMinute = 0.0;

    public double frontMotorAppliedVolts = 0.0;
    public double rearMotorAppliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  ;
  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
  ;
}
