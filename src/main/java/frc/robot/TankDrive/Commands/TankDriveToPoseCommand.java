package frc.robot.TankDrive.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimToPoseRotationPIDNT;
import frc.robot.AimToPoseTranslationPIDNT;
import frc.robot.TankDrive.TankSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

/**
 * 三步走：转向目标点 → 直线开（可倒）→ 转最终角
 * 死区强制停车 + 关掉 I 项，防止到位后晃
 */
public class TankDriveToPoseCommand extends Command {

    private final TankSubsystem tank;
    private final Supplier<Pose2d> robotPose;
    private final Supplier<Pose2d> targetPose;
    private final PIDController xController;
    private final PIDController thetaController;
    private final Distance transTol;
    private final Angle angleTol;

    private enum State { TURN_TO_POINT, DRIVE_TO_POINT, TURN_TO_FINAL }
    private State state = State.TURN_TO_POINT;

    private double dx, dy, dist, angleToPoint, angleErr;

    public TankDriveToPoseCommand(TankSubsystem tank,
                                  Supplier<Pose2d> robotPose,
                                  Supplier<Pose2d> targetPose,
                                  PIDController xController,
                                  PIDController thetaController,
                                  Distance transTol,
                                  Angle angleTol) {
        this.tank            = tank;
        this.robotPose       = robotPose;
        this.targetPose      = targetPose;
        this.xController     = xController;
        this.thetaController = thetaController;
        this.transTol        = transTol;
        this.angleTol        = angleTol;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(tank);
    }

    @Override
    public void initialize() {
        xController.reset();
        thetaController.reset();
        thetaController.setI(0);          // ****** 关掉 I，防止积分晃 ******
        state = State.TURN_TO_POINT;
    }

    @Override
    public void execute() {
        xController.setPID(AimToPoseTranslationPIDNT.kP.getValue(),
                        AimToPoseTranslationPIDNT.kI.getValue(),
                        AimToPoseTranslationPIDNT.kD.getValue());
        thetaController.setPID(AimToPoseRotationPIDNT.kP.getValue(),
                            AimToPoseRotationPIDNT.kI.getValue(),
                            AimToPoseRotationPIDNT.kD.getValue());

        Pose2d cur  = robotPose.get();
        Pose2d goal = targetPose.get();

        dx           = goal.getX() - cur.getX();
        dy           = goal.getY() - cur.getY();
        dist         = Math.hypot(dx, dy);
        angleToPoint = Math.atan2(dy, dx);

        /* ****** 只归一化一次，交给 enableContinuousInput 走最短弧 ******/
        angleErr = thetaController.calculate(cur.getRotation().getRadians(), angleToPoint);

        switch (state) {
            case TURN_TO_POINT:
                System.out.printf("TURN angleErr=%.4f  tol=%.4f%n", angleErr, angleTol.in(Radians));

                /* ****** 放大死区 + 强制停车 ******/
                if (Math.abs(angleErr) < 0.08) {          // 0.08 rad ≈ 5°
                    tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, 0)));
                    state = State.DRIVE_TO_POINT;
                    break;
                }

                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, angleErr)));
                break;

            case DRIVE_TO_POINT:
                double v = xController.calculate(dist, 0);
                v = -v;                       // 你要的倒着走
                double omega = thetaController.calculate(cur.getRotation().getRadians(), angleToPoint);
                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(v, 0, omega)));

                if (dist < transTol.in(Meters)) {
                    state = State.TURN_TO_FINAL;
                }
                break;

            case TURN_TO_FINAL:
                angleErr = thetaController.calculate(cur.getRotation().getRadians(), goal.getRotation().getRadians());
                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, angleErr)));
                if (Math.abs(angleErr) < 0.08) {
                    end(false);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.TURN_TO_FINAL &&
               Math.abs(robotPose.get().getRotation().getRadians() - targetPose.get().getRotation().getRadians())
               < angleTol.in(Radians);
    }

    @Override
    public void end(boolean interrupted) {
        tank.runStop();
    }
}