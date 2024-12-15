package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedBucketAuto", group="RedSide")
public class NewAutoRedBucket extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "rightSlide");
            lift2 = hardwareMap.get(DcMotorEx.class, "leftSlide");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class LiftUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-1);
                    lift2.setPower(-1);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 11594) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }

    private final testHardware robot = new testHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot.init(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Vector2d basketPos = new Vector2d(-56, -60);
        double slideInitPos = robot.rightSlide.getCurrentPosition();
        robot.clawRotate.setPosition(1);
        robot.claw.setPosition(0);
        waitForStart();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Right slide current position: ", robot.rightSlide.getCurrentPosition());

        if (!isStopRequested() && opModeIsActive()) {
            TrajectoryActionBuilder action = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-34, -42))
                    .waitSeconds(0.5)
                    .turn(Math.toRadians(-45))
                    .waitSeconds(0.5)
                    .strafeTo(basketPos)
                    .waitSeconds(0.5);
            TrajectoryActionBuilder action2 = drive.actionBuilder(drive.pose)
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(-60, -64), Math.toRadians(45));
            TrajectoryActionBuilder action3 = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(-45, -50), Math.toRadians(90));
            TrajectoryActionBuilder waitingAction = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5);
            //drop preload
//                    .turn(Math.toRadians(45))
//                    .strafeTo(new Vector2d(-48.5, -32)) //grab first sample
//                    .strafeTo(basketPos)
//                    .turn(Math.toRadians(-45))
//                    //drop sample
//                    .turn(Math.toRadians(45))
//                    .strafeTo(new Vector2d(-57, -32))
//                    .strafeTo(basketPos)
//                    .turn(Math.toRadians(-45))
//                    //drop sample
//                    .turn(Math.toRadians(45))
////                THIRD SAMPLE
////                .lineToY(-26)
////                .turn(Math.toRadians(90))
////                .lineToX(-60)
////                .turn(Math.toRadians(-90))
////                .strafeTo(basketPos)
////                .turn(Math.toRadians(-45))
//                    //drop sample
//                    .splineToLinearHeading(new Pose2d(-22, 0, Math.toRadians(-180)), Math.PI/2);
            Actions.runBlocking(new SequentialAction(action.build(), lift.liftUp()));
//            while (robot.rightSlide.getCurrentPosition() > (slideInitPos - 11594)) {
//                robot.rightSlide.setPower(-1);
//                robot.leftSlide.setPower(-1);
//            }
//            robot.rightSlide.setPower(0);
//            robot.leftSlide.setPower(0);

            Actions.runBlocking(action2.build());

            Servo release = hardwareMap.get(Servo.class, "release");
            robot.release.setPosition(0.999);

            Thread.sleep(500);

            Actions.runBlocking(new SequentialAction(action3.build(), waitingAction.build()));

            double i = runtime.milliseconds();
            while (runtime.milliseconds() < i + 250) robot.intakeArm.setPower(0.5);
            robot.intakeArm.setPower(0);

            double g = runtime.milliseconds();
            double gaver;
            while (getRuntime() - g < 0.5) {
                gaver = 0;
            }
            double j = runtime.milliseconds();
            while (runtime.milliseconds() < j + 250) robot.claw.setPosition(1);

            double d = runtime.milliseconds();
            double daver;
            while (getRuntime() - d < 0.5) {
                daver = 0;
            }
            double k = runtime.milliseconds();
            while (runtime.milliseconds() < k + 250) robot.clawRotate.setPosition(1);

            double r = runtime.milliseconds();
            double raver;
            while (getRuntime() - r < 0.5) {
                raver = 0;
            }
            double v = runtime.milliseconds();
            while (runtime.milliseconds() < v + 250) robot.intakeArm.setPower(-0.3);
            robot.intakeArm.setPower(0);

            double w = runtime.milliseconds();
            double waver;
            while (getRuntime() - w < 0.5) {
                waver = 0;
            }
            double u = runtime.milliseconds();
            while (robot.rightSlide.getCurrentPosition() != slideInitPos) {
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
            }

        }

    }
}
