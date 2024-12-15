package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedBucketAuto", group="RedSide")
public class NewAutoRedBucket extends LinearOpMode{
    private final testHardware robot = new testHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34,-62,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot.init(hardwareMap);
        Vector2d basketPos = new Vector2d(-56, -60);
        double slideInitPos = robot.rightSlide.getCurrentPosition();

        waitForStart();
        robot.clawRotate.setPosition(1);
        robot.claw.setPosition(0);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Right slide current position: ", robot.rightSlide.getCurrentPosition());

        if (!isStopRequested()&&opModeIsActive()){
            TrajectoryActionBuilder action = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-34,-42))
                    .waitSeconds(0.5)
                    .turn(Math.toRadians(-45))
                    .waitSeconds(0.5)
                    .strafeTo(basketPos)
                    .waitSeconds(0.5);
            TrajectoryActionBuilder action2 = drive.actionBuilder(drive.pose)
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(-60,-64), Math.toRadians(45));
            TrajectoryActionBuilder action3 = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(-46, -51), Math.toRadians(90));
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
            Actions.runBlocking(action.build());
            while (robot.rightSlide.getCurrentPosition() > (slideInitPos-11594)){
                robot.rightSlide.setPower(-1);
                robot.leftSlide.setPower(-1);
            }robot.rightSlide.setPower(0);
            robot.leftSlide.setPower(0);
            Actions.runBlocking(action2.build());
            Servo release = hardwareMap.get(Servo.class, "release");
            robot.release.setPosition(0.999);
            Actions.runBlocking(waitingAction.build());
            Actions.runBlocking(action3.build());
            robot.rightSlide.setPower(1);
            robot.leftSlide.setPower(1);
            Actions.runBlocking(waitingAction.build());
            robot.intakeArm.setPower(0);
            robot.claw.setPosition(1);
            Actions.runBlocking(waitingAction.build());
            robot.clawRotate.setPosition(0);
            robot.intakeArm.setPower(-1);
            Actions.runBlocking(waitingAction.build());
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);

        }

    }
}
