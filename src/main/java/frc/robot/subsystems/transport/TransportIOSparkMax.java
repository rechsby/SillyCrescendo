package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.TransportConstants;

public class TransportIOSparkMax implements TransportIO {
  private CANSparkMax transportMotor = new CANSparkMax(0, MotorType.kBrushless);
  private DigitalInput breakBeam = new DigitalInput(9);
  private boolean breakBeamInvert = true;

  @Override
  public void updateInputs(TransportIOInputs inputs) {
    inputs.transportRotationsPerMinute =
        transportMotor.getEncoder().getVelocity() / TransportConstants.GEAR_RATIO;
    inputs.transportMotorAppliedVolts =
        transportMotor.getAppliedOutput() * transportMotor.getBusVoltage();
    inputs.hasNote = breakBeam.get() ^ breakBeamInvert;
    inputs.currentAmps = new double[] {transportMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    transportMotor.setVoltage(volts);
  }
}
