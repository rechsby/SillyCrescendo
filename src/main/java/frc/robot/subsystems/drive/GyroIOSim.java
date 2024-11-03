package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = this.gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = this.gyroSimulation.getMeasuredAngularVelocityRadPerSec();
  }
}
// public interface GyroIO {
//   @AutoLog
//   public static class GyroIOInputs {
//     public boolean connected = false;
//     public Rotation2d yawPosition = new Rotation2d();
//     public double yawVelocityRadPerSec = 0.0;
//   }

//   public default void updateInputs(GyroIOInputs inputs) {}
// }
