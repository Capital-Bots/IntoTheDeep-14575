package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Basic All Directions OpMode", group="Iterative OpMode")
@Disabled
public class BasicStrafeOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    boolean up;
    boolean left;
    boolean right;
    boolean down;

    double leftFrontDrivePower;
    double rightFrontDrivePower;
    double leftBackDrivePower;
    double rightBackDrivePower;

    double speedval = 0.45;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        up = gamepad1.dpad_up;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;
        down = gamepad1.dpad_down;

        if (up) {
            leftFrontDrivePower = speedval;
            leftBackDrivePower = speedval;
            rightFrontDrivePower = speedval;
            rightBackDrivePower = speedval;
        }
        if (down) {
            leftFrontDrivePower = -1 * speedval;
            leftBackDrivePower = -1 * speedval;
            rightFrontDrivePower = -1 * speedval;
            rightBackDrivePower = -1 * speedval;
        }
        if (right) {
            leftFrontDrivePower = speedval;
            leftBackDrivePower = -1 * speedval;
            rightFrontDrivePower = -1 * speedval;
            rightBackDrivePower = speedval;
        }
        if (left) {
            leftFrontDrivePower = -1 * speedval;
            leftBackDrivePower = speedval;
            rightFrontDrivePower = speedval;
            rightBackDrivePower = -1 * speedval;
        }

        leftFrontDrive.setPower(leftFrontDrivePower);
        leftBackDrive.setPower(leftBackDrivePower);
        rightFrontDrive.setPower(rightFrontDrivePower);
        rightBackDrive.setPower(rightBackDrivePower);
    }
}
