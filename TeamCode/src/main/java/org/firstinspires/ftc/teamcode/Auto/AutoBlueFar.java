package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.opencv.OpenCVBlue.getDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.OpenCVBlue;
import org.firstinspires.ftc.teamcode.opencv.OpenCVBlue.blueBlobDetectionPipeline;
import org.firstinspires.ftc.teamcode.opencv.OpenCVRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Auto Close to Board", group = "BlueSide")
public class AutoBlueFar extends LinearOpMode{
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    public DcMotor rightSlideRotate = null;
    public DcMotor leftSlideRotate  = null;
    public DcMotorEx leftEncoder    = null;
    public DcMotorEx rightEncoder   = null;
    public int location = 2;
    @Override

    public void runOpMode(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        rightSlideRotate = hardwareMap.get(DcMotor.class, "rightSlideRotate");
        leftSlideRotate = hardwareMap.get(DcMotor.class, "leftSlideRotate");

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        while (!opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) OpenCVBlue.cX + ", " + (int) OpenCVBlue.cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(OpenCVBlue.width)));

            if(OpenCVBlue.cX < CAMERA_WIDTH/3 && OpenCVBlue.cX >= 0){
                location = 0;
            } else if(OpenCVBlue.cX >= CAMERA_WIDTH/3 && OpenCVBlue.cX < 2*CAMERA_WIDTH/3){
                location = 1;
            } else{
                location = 2;
            }


            telemetry.addData("Location: ",location);
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();



        waitForStart();



        if (opModeIsActive()) {
            if (location ==1){
                while (leftEncoder.getCurrentPosition() > -1 * 52500){
                    leftFrontDrive.setPower(-0.3);
                    rightFrontDrive.setPower(-0.3);
                    leftBackDrive.setPower(-0.3);
                    rightBackDrive.setPower(-0.3);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                }
            }
            else if (location == 2) {
                while (leftEncoder.getCurrentPosition() > -1 * 20000){
                    leftFrontDrive.setPower(-0.3);
                    rightFrontDrive.setPower(-0.3);
                    leftBackDrive.setPower(-0.3);
                    rightBackDrive.setPower(-0.3);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                }
                while (leftEncoder.getCurrentPosition() > -1 * 28000){
                    leftFrontDrive.setPower(-0.1);
                    rightFrontDrive.setPower(-0.3);
                    leftBackDrive.setPower(-0.1);
                    rightBackDrive.setPower(-0.3);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                }
            }
            else if (location == 0){
                while (leftEncoder.getCurrentPosition() > -1 * 35000){
                    leftFrontDrive.setPower(-0.3);
                    rightFrontDrive.setPower(-0.3);
                    leftBackDrive.setPower(-0.3);
                    rightBackDrive.setPower(-0.3);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                }
                while (leftEncoder.getCurrentPosition() > -1 * 70000){
                    leftFrontDrive.setPower(-0.3);
                    rightFrontDrive.setPower(-0.1);
                    leftBackDrive.setPower(-0.3);
                    rightBackDrive.setPower(-0.1);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                }
            }
        }
    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new blueBlobDetectionPipeline());

        controlHubCam.openCameraDevice(); //I don't know why this is deprecated
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
