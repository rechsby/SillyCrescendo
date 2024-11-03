package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void intake() {
    io.setVoltage(12.0);
  }

  public void stop() {
    io.setVoltage(0);
  }

  public Command startIntake() {
    return Commands.runOnce(
        () -> {
          intake();
        },
        this);
  }

  public Command stopIntake() {
    return Commands.runOnce(
        () -> {
          stop();
        },
        this);
  }
}
