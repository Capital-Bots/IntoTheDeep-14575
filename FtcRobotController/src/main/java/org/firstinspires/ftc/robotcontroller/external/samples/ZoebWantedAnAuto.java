package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Zoeb Wanted an Auto So I Made One", group = "Robot")
public class ZoebWantedAnAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

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


        leftFrontDrivePower = speedval;
        leftBackDrivePower = speedval;
        rightBackDrivePower = speedval;
        rightFrontDrivePower = speedval;

        while (runtime.seconds()<2){
            leftFrontDrive.setPower(leftFrontDrivePower);
            leftBackDrive.setPower(leftBackDrivePower);
            rightFrontDrive.setPower(rightFrontDrivePower);
            rightBackDrive.setPower(rightBackDrivePower);
        }


        leftFrontDrivePower = speedval;
        leftBackDrivePower = -1*speedval;
        rightBackDrivePower = speedval;
        rightFrontDrivePower = -1*speedval;

        while ((runtime.seconds()-2)<2){
            leftFrontDrive.setPower(leftFrontDrivePower);
            leftBackDrive.setPower(leftBackDrivePower);
            rightFrontDrive.setPower(rightFrontDrivePower);
            rightBackDrive.setPower(rightBackDrivePower);
        }


        leftFrontDrivePower = -1*speedval;
        leftBackDrivePower = speedval;
        rightBackDrivePower = -1*speedval;
        rightFrontDrivePower = speedval;

        while ((runtime.seconds()-4)<2){
            leftFrontDrive.setPower(leftFrontDrivePower);
            leftBackDrive.setPower(leftBackDrivePower);
            rightFrontDrive.setPower(rightFrontDrivePower);
            rightBackDrive.setPower(rightBackDrivePower);
        }


        leftFrontDrivePower = -1*speedval;
        leftBackDrivePower = -1*speedval;
        rightBackDrivePower = -1*speedval;
        rightFrontDrivePower = -1*speedval;

        while ((runtime.seconds()-6)<2){
            leftFrontDrive.setPower(leftFrontDrivePower);
            leftBackDrive.setPower(leftBackDrivePower);
            rightFrontDrive.setPower(rightFrontDrivePower);
            rightBackDrive.setPower(rightBackDrivePower);
        }
    }
}
