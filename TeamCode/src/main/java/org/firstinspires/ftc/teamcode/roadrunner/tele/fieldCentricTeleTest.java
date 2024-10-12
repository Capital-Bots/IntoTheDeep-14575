package org.firstinspires.ftc.teamcode.roadrunner.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(group = "drive", name = "fieldcenttest")
public class fieldCentricTeleTest extends LinearOpMode {
//     double forwardHeading = Math.toRadians(180);
    private final testHardware robot = new testHardware();
    boolean isResetRequested;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        robot.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {



            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y
            );



            double robotHeading = drive.pose.heading.toDouble();
            double xComp = input.x;
            double yComp = -1 * input.y;
            double inputHeading = Math.atan2(yComp, xComp);
            double inputMagnitude = Math.sqrt(xComp*xComp + yComp*yComp);
            inputHeading -= robotHeading;
            isResetRequested = gamepad1.a;

            double finalX = inputMagnitude * Math.cos(inputHeading);
            double finalY = inputMagnitude * Math.sin(inputHeading);
            double turnComponent = gamepad1.right_stick_x;
            double SPEED_MULTIPLIER = 0.9;
            double normalizingFactor = Math.max(Math.abs(finalY)
                    + Math.abs(finalX) + Math.abs(turnComponent), 1);

            if (isResetRequested){
                drive.pose = new Pose2d(new Vector2d(0,0), 0);
            }

            double fl = SPEED_MULTIPLIER * (finalY + finalX + turnComponent) / normalizingFactor;
            double fr = SPEED_MULTIPLIER * (finalY - finalX - turnComponent) / normalizingFactor;
            double bl = SPEED_MULTIPLIER * (finalY - finalX + turnComponent) / normalizingFactor;
            double br = SPEED_MULTIPLIER * (finalY + finalX - turnComponent) / normalizingFactor;

            drive.updatePoseEstimate();
            robot.leftFrontDrive.setPower(fl);
            robot.leftBackDrive.setPower(bl);
            robot.rightFrontDrive.setPower(fr);
            robot.rightBackDrive.setPower(br);
            telemetry.addData("heading", robotHeading);
            telemetry.addData("input heading", inputHeading);
            telemetry.addData("x-comp", input.x);
            telemetry.addData("y-comp", input.y);
            telemetry.update();
        }
    }
}