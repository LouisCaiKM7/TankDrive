package frc.robot.TankDrive.Side;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.TankParamsNT;
import frc.robot.TankDrive.TankConfigs;
import frc.robot.TankDrive.TankConfigs.SideConfigs;

public class Side {
    private final SideIO io;
    private final SideIOInputsAutoLogged[] data;

    private final SideConfigs configs;
    private final int motorNum;
    private final Distance wheelDiameter;

    public Side(SideConfigs configs, SideIO io) {
        this.configs = configs;
        this.io = io;
        this.motorNum = configs instanceof TankConfigs.SideConfigsReal ?
        ((TankConfigs.SideConfigsReal) configs).motorNumber() :
        ((TankConfigs.SideConfigsSim) configs).motorNumber();
        this.wheelDiameter = configs instanceof TankConfigs.SideConfigsReal ?
        ((TankConfigs.SideConfigsReal) configs).wheelDiameter :
        ((TankConfigs.SideConfigsSim) configs).wheelDiameter;
        this.data = new SideIOInputsAutoLogged[motorNum];
        for(int i = 0; i < motorNum; i++){
            this.data[i] = new SideIOInputsAutoLogged();
        }
        updateDriveController();
    }

     

    public void updateInputs() {
        io.updateInputs(data);
        for(int i = 0; i < motorNum; i++){
            Logger.processInputs(configs.getName() + "/Motor/" + (i+1), data[i]);
        }
    }


    public void updateDriveController() {
        double kp = TankParamsNT.kP.getValue();
        double ki = TankParamsNT.kI.getValue();
        double kd = TankParamsNT.kD.getValue();
        double ks = TankParamsNT.kS.getValue();
        double kv = TankParamsNT.kV.getValue();
        double ka = TankParamsNT.kA.getValue();
        io.configSideController(kp, ki, kd, ks, kv, ka);
    }

    public void periodic() {
        io.updateInputs(data);
        if(TankParamsNT.isAnyChanged()){
            updateDriveController();
        }
        Logger.recordOutput(configs.getName(), TankParamsNT.isAnyChanged());
    }

    public void runVoltage(Voltage voltage){
        io.setDriveOpenLoop(voltage);
    }


    public void runVelocity(LinearVelocity velocity){
        io.setDriveVelocity(velocity);
    }

    public void runVelocity(LinearVelocity velocity, Current ff){
        io.setDriveVelocity(velocity, ff);
    }

    public void runStop(){
        runVelocity(MetersPerSecond.zero());
    }

    public Distance getDriveDistance() {
        double totalDistance = 0;
        for(int i = 0; i < motorNum; i++){
            totalDistance += data[i].sideMotorPositionRad;
        }
        double avgDistance = totalDistance / motorNum;
        return wheelDiameter.times(avgDistance * 0.5);
    }

    public LinearVelocity getVelocity(){
        double totalVelocity = 0;
        for(int i = 0; i < motorNum; i++){
            totalVelocity += data[i].sideMotorVelocityRadPerSec;
        }
        double avgVelocity = totalVelocity / motorNum;
        return MetersPerSecond.of(avgVelocity * 0.5 * wheelDiameter.in(Meters));
    }

    public Distance[] getSampledSideDistances() {
        int sampleCount = data[0].sideMotorPositionRadSamples.length;
        int temp;
        for(int i = 0; i < motorNum; i++){
            temp = data[i].sideMotorPositionRadSamples.length;
            if (sampleCount > temp) {
                sampleCount = temp;
            }
            else{
                continue;
            }
        }
        Distance[] distances = new Distance[sampleCount];
        double distance = 0.0;
        double totalDistance = 0.0;
        for (int i = 0; i < sampleCount; i++){
            for(int j = 0; j < motorNum; j++){
                distance = data[j].sideMotorPositionRadSamples[i];
                totalDistance += distance;
            }
            distances[i] = wheelDiameter.times(totalDistance / motorNum * 0.5);
            totalDistance = 0.0;
        }
            
        return distances;
    }
}
