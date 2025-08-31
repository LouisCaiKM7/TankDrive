package frc.robot.TankDrive.Imu;
//Finished, if anychange needed, please contact, rather than just editing the file.
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;

public class ImuIOSim implements ImuIO {
    public ImuIOSim() {

    }

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = inputs.yawPosition.plus(new Rotation2d(inputs.yawVelocityRadPerSecCmd * 0.3));
        inputs.yawVelocityRadPerSec = inputs.yawVelocityRadPerSecCmd;
        inputs.pitchPosition = new Rotation2d();
        inputs.pitchVelocityRadPerSec = 0.0;
        inputs.rollPosition = new Rotation2d();
        inputs.rollVelocityRadPerSec = 0.0;
        inputs.odometryYawTimestamps = new double[]{Timer.getTimestamp()};
        inputs.odometryYawPositions = new Rotation2d[]{inputs.yawPosition};
        inputs.odometryRotations = new Rotation3d[]{
                new Rotation3d(
                        inputs.rollPosition.getMeasure(),
                        inputs.pitchPosition.getMeasure(),
                        inputs.yawPosition.getMeasure()
                )
        };
    }
}
