package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

/*
 This OpMode is meant to to test certain parts of the Odometry.java class without the use of dead wheels
 or a robot. It will use gamepads to input values, and telemetry to output the results of certain
 positioning algorithms so that the input and output are correct. It is currently using an overhauled
 locate() method from the Odometry.java class because the new method is being tested for improvements.
 If the results are positive, we will add this locate() method to the Odometry.java class.
 You can just ignore the warning signs, they don't affect the program.
 */

@TeleOp(name = "TestingOdometry")
public class TestingOdometry extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    boolean detectionFound = false;
    double tagDis = 0;


    private DcMotor fl, fr, bl, br;
    double robotX = 0, robotY = 0;  // Starting X and Y position of the robot
    /*               ^
                     ^
    I am planning on using april tags to determine position on the field
    and change starting x and y values

     */
    private double previousHeading = 0;

    final int TICKS_PER_INCH = 44;

    double DRIVE_SPEED;
    final double TOLERANCE = 0.5;
    final double HEADING_TOLERANCE = 1;
    double distanceToTarget = 0;

    double kP = 0.0135;// TUNE THIS!!! THIS IS JUST A GUESS!!!!!!

    double leftEncoder = 0;
    double rightEncoder = 0;
    double centerEncoder = 0;

    private double leftPosition = 0;
    private double rightPosition = 0;

    private boolean highBasket = false;
    private boolean reach = false;
    private double headingError;

    double currentYPosition = 0;
    double currentXPosition = 0;

    @Override
    public void runOpMode() {

        //Initialize sensors
        initAprilTag();

        //Map to configurations
        fl = hardwareMap.get(DcMotor.class, "fl");
        fl.setDirection(DcMotor.Direction.REVERSE);

        bl = hardwareMap.get(DcMotor.class, "bl");
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr = hardwareMap.get(DcMotor.class, "fr");

        br = hardwareMap.get(DcMotor.class, "br");



        resetEncoders();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("OpMode Running");
            telemetry.update();
            double vert = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            leftEncoder += vert;
            rightEncoder += vert;
            centerEncoder += horizontal;

            locate();
            telemetry.addData("robotX", robotX);
            telemetry.addData("robotY", robotY);

        }
        if(!opModeIsActive()){
            stop();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()





    private void resetEncoders(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }





    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()






    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 1) {
                detectionFound = true;
                tagDis = detection.ftcPose.range;
                telemetry.addData("Distance: ", tagDis );
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()





    /**
     * Localization method, will be constantly called to ensure accurate positions
     */
    private void locate() {



        double deltaLeftPos = (leftEncoder / TICKS_PER_INCH) - leftPosition;
        double deltaRightPos = (rightEncoder / TICKS_PER_INCH) - rightPosition;

        // Read encoder values (convert to inches)
        leftPosition = leftEncoder / TICKS_PER_INCH;
        rightPosition = rightEncoder / TICKS_PER_INCH;
        double centerPosition = centerEncoder / TICKS_PER_INCH;

        double ignorePivoting = (deltaLeftPos - deltaRightPos) * 1.5;//TUNE: totally random



        double deltaPositionY = ((leftPosition + rightPosition)/2) - currentYPosition;
        currentYPosition = (leftPosition + rightPosition) / 2;
        double deltaPositionX = centerPosition - ignorePivoting;

        double deltaHeadingDegrees = (leftPosition - rightPosition) * 7;//TUNE: the number is degrees per inch
        double deltaHeading = Math.toRadians(deltaHeadingDegrees);

        // Calculate the lateral movement of the center wheel with correction for its offset


        /**   NOTE: Calculate the new position using trigonometry (X and Y position update)**/
        robotX += Math.cos(previousHeading) * deltaPositionX;
        robotY += Math.sin(previousHeading) * deltaPositionY;

        // Update the previous heading
        previousHeading += deltaHeading;
        double robotHeading = Math.toDegrees(previousHeading);
    }







}   // end class