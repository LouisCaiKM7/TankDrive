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
 * 严格符号版：
 * 顺时针为负，逆时针为正
 * case 行为完全匹配你给的表格
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

    /* 每周期复用 */
    private double dx, dy, dist, angleToPoint;
    /* 当前选的方向标志：false=正向（v=-v），true=反向（v=+v） */
    private boolean driveReverse;

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
        thetaController.setI(0);
        state        = State.TURN_TO_POINT;
        driveReverse = false;
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
        angleToPoint = Math.atan2(dy, dx);   // 逆时针为正，顺时针为负

        switch (state) {
            case TURN_TO_POINT:
                double rawErr = angleToPoint - cur.getRotation().getRadians();
                /* ****** 用 ±90° 界限判断哪边更近 ******/
                if (Math.abs(rawErr) > Math.PI / 2) {          // >90° 才考虑倒车
                    double revErr = rawErr - Math.copySign(Math.PI, rawErr);   // 朝反方向
                    if (Math.abs(revErr) < Math.abs(rawErr)) {
                        driveReverse = true;
                        angleToPoint = angleToPoint - Math.copySign(Math.PI, rawErr);   // 翻目标线
                    } else {
                        driveReverse = false;   // 尽管>90°，但原方向更近
                    }
                } else {
                    driveReverse = false;       // ≤90° 直接正走
                }

                double angleErr = thetaController.calculate(cur.getRotation().getRadians(), angleToPoint);
                System.out.printf("TURN angleErr=%.4f  rev=%s  tol=%.4f%n", angleErr, driveReverse, angleTol.in(Radians));

                if (Math.abs(angleErr) < angleTol.in(Radians)) {
                    tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, 0)));
                    state = State.DRIVE_TO_POINT;
                    break;
                }

                double omega = thetaController.calculate(cur.getRotation().getRadians(), angleToPoint);
                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, omega)));
                break;
            case DRIVE_TO_POINT:
                double v = xController.calculate(dist, 0);
                /* 根据选的方向给符号：reverse=true → 正速度（倒着开） */
                v = driveReverse ? v : -v;

                /* 行驶中保持车头朝目标线 */
                double omegaDrive = thetaController.calculate(cur.getRotation().getRadians(), angleToPoint);
                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(v, 0, 0)));

                if (dist < transTol.in(Meters)) {
                    state = State.TURN_TO_FINAL;
                }
                break;

            case TURN_TO_FINAL:
                /* ****** 转到最终目标角 ******/
                double finalErr = thetaController.calculate(cur.getRotation().getRadians(), goal.getRotation().getRadians());
                tank.runVelocity(tank.getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, finalErr)));
                if (Math.abs(finalErr) < angleTol.in(Radians)) {
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