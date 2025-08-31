package frc.robot.TankDrive;
//Finished, if anychange needed, please contact, rather than just editing the file.
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Builder;

@Builder
public record TankLimits(LinearVelocity maxLinearVelocity, LinearAcceleration maxLinearAcceleration){
    public Pair<DifferentialDriveWheelSpeeds, Pair<LinearAcceleration, LinearAcceleration>> apply(DifferentialDriveWheelSpeeds currentSpeed,
                                              DifferentialDriveWheelSpeeds desiredSpeed, double dt){
        
        double leftSpeedLimit = maxLinearVelocity.magnitude();
        double rightSpeedLimit = maxLinearVelocity.magnitude();

        double leftAccelerationLimit = maxLinearAcceleration.magnitude();
        double rightAccelerationLimit = maxLinearAcceleration.magnitude();

        double leftSpeed = currentSpeed.leftMetersPerSecond;
        double rightSpeed = currentSpeed.rightMetersPerSecond;

        double desiredleftSpeed = desiredSpeed.leftMetersPerSecond;
        double desiredRightSpeed = desiredSpeed.rightMetersPerSecond;
        
        double limitedLeftSpeed =  Math.min(Math.max(leftSpeed, -leftSpeedLimit), leftSpeedLimit);
        double limitedRightSpeed = Math.min(Math.max(rightSpeed, -rightSpeedLimit), rightSpeedLimit);

        double leftAcceleration = (desiredleftSpeed - leftSpeed)/dt;
        double rightAcceleration = (desiredRightSpeed - rightSpeed)/dt;

        double limitedLeftAcceleration = Math.min(Math.max(leftAcceleration, -leftAccelerationLimit), leftAccelerationLimit);
        double limitedRightAcceleration = Math.min(Math.max(rightAcceleration, -rightAccelerationLimit), rightAccelerationLimit);

        DifferentialDriveWheelSpeeds limitedSpeeds = new DifferentialDriveWheelSpeeds(limitedLeftSpeed, limitedRightSpeed);
        Pair<LinearAcceleration, LinearAcceleration> limitedAccelerations = new Pair<>(
            LinearAcceleration.ofBaseUnits(limitedLeftAcceleration, MetersPerSecondPerSecond),
            LinearAcceleration.ofBaseUnits(limitedRightAcceleration, MetersPerSecondPerSecond)
        );

        return Pair.of(limitedSpeeds, limitedAccelerations);
    }


}
