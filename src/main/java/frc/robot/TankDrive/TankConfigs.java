package frc.robot.TankDrive;
//Finished, if anychange needed, please contact, rather than just editing the file.
import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.SuperBuilder;

@Getter
@SuperBuilder
public class TankConfigs {
    public final String name;

    public TankLimits tankLimits;


    public final double trackWidth;

    public Pair<SideConfigs, SideConfigs> sides;

    public interface SideConfigs {
        String getName();
        int motorNumber();
    }

    @Getter
    @Builder
    public static class SideConfigsReal implements SideConfigs {
        public String name;
        public final double deltaTimeInSeconds;
        public double gearRatio;
        public final int pigeonId;
        public Frequency odometryFrequency;
        public String canivoreCanBusName;

        public ArrayList<TalonFX> motors;
        public ArrayList<Integer> motorId;
        public ArrayList<Boolean> motorInverted;
        public Current driveStatorCurrentLimit;
        public final Distance wheelDiameter;

        @Override
        public int motorNumber() {
            return motors.size();
        }
    }



    @Getter
    @Builder
    public static class SideConfigsSim implements SideConfigs {
        public String name;
        public final double deltaTimeInSeconds;
        public double gearRatio;
        public final Distance wheelDiameter;

        public Distance distanceRelativeToCenter;
        public ArrayList<DCMotor> motors;
        public ArrayList<MomentOfInertia> momentOfInertia;
        public ArrayList<Boolean> motorInverted;
        public double driveStdDevPos;
        public double driveStdDevVel;

        @Override
        public int motorNumber() {
            return motors.size();
        }
    }

}
