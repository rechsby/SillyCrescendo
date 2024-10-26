package frc.robot.subsystems.transport;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.TransportConstants;

public class TransportIOSim implements TransportIO {
  private FlywheelSim transportMotorSim =
      new FlywheelSim(DCMotor.getNEO(1), TransportConstants.GEAR_RATIO, 0.004);
  private double appliedVolts = 0;

  @Override
  public void updateInputs(TransportIOInputs inputs) {
    inputs.transportRotationsPerMinute =
        transportMotorSim.getAngularVelocityRPM() / TransportConstants.GEAR_RATIO;
    inputs.transportMotorAppliedVolts = appliedVolts;
    inputs.hasNote = false; // TODO: How do i do break beam in sim?
    inputs.currentAmps = new double[] {transportMotorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = 0;
    transportMotorSim.setInputVoltage(volts);
  }
}
