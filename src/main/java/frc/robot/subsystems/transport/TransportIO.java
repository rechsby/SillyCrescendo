package frc.robot.subsystems.transport;

import org.littletonrobotics.junction.AutoLog;

public interface TransportIO {
  @AutoLog
  public static class TransportIOInputs {
    public double transportRotationsPerMinute = 0.0;
    public double transportMotorAppliedVolts = 0.0;
    public boolean hasNote = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(TransportIOInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;
  ;
}
