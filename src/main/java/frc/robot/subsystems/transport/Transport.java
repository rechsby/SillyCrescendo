package frc.robot.subsystems.transport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;
import org.littletonrobotics.junction.Logger;

public class Transport extends SubsystemBase {
  private final TransportIO io;
  private final TransportIOInputsAutoLogged inputs = new TransportIOInputsAutoLogged();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transport", inputs);
  }

  public Transport(TransportIO io) {
    this.io = io;
  }

  public void transport() {
    runPercentage(TransportConstants.TRANSPORT_SPEED);
  }

  public void stopTransport() {
    io.setVoltage(0);
  }

  public void runPercentage(double percentage) {
    io.setVoltage(MathUtil.clamp((percentage * 12.0), -12, 12));
  }

  public boolean isNotePresent() {
    return inputs.hasNote;
  }
}
