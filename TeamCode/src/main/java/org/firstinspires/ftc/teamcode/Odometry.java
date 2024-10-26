package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
 This OpMode is an untested attempt at using dead wheels for localization on the game field.
 It uses trigonometry to calculate angles and motor powers to reach certain points
 Starting points are to be inserted in the robotX and robotY initialization, starting heading is put into previousHeading
 The driveTo method will drive to any point on the field, and the driveToWithHeading will change the heading while moving to target
 Certain values and variables will need to be tuned, and are specifically marked
 */
@Autonomous(name = "AutoMethods")
public class Odometry extends LinearOpMode {

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
    private DcMotor base, arm, slide;
    private Servo wrist,claw;
    private DcMotorEx leftEncoder, rightEncoder, centerEncoder;
    double robotX = 0, robotY = 0;  // Starting X and Y position of the robot
    /*               ^
                     ^
    I am planning on using april tags to determine position on the field
    and change starting x and y values

     */
    private double previousHeading = 0;

    final int TICKS_PER_INCH = 44;
    final double WHEEL_BASE = 16.0;
    final double CENTER_WHEEL_OFFSET = 2.0;//Tune based on distance

    double DRIVE_SPEED;
    final double TOLERANCE = 0.5;
    final double HEADING_TOLERANCE = 1;
    double distanceToTarget = 0;

    final double kP = 0.0135;// TUNE THIS!!! THIS IS JUST A GUESS!!!!!!

    private boolean highBasket = false;
    private boolean reach = false;
    private double headingError;

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

        base = hardwareMap.get(DcMotor.class, "base");
        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightEncoder = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerEncoder = hardwareMap.get(DcMotorEx.class, "centerEncoder");
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        resetEncoders();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()){
            telemetry.addLine("OpMode Running");
            telemetry.update();

            //These are just examples of what the methods look like when called
            open();
            reach = true;
            driveToWithHeading(0,0, 90);
            driveTo(100,100);
            driveToWithHeading(50,50,0);
            driveTo(150,150);
            highBasket = true;
            close();
            telemetryAprilTag();

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





    //Halt motors
    private void halt(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    /**
     * WARNING: Completely incorrect algorithms. New method in development.
     * New method can be accessed in the locate() method in the TestingOdometry.java class
     */
    private void locate() {


        // Read encoder values (convert to inches)
        double leftPosition = (double) leftEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double rightPosition = (double) rightEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double centerPosition = (double) centerEncoder.getCurrentPosition() / TICKS_PER_INCH;

        // Calculate heading based on wheel positions
        double deltaLeft = leftPosition - previousHeading;
        double deltaRight = rightPosition - previousHeading;
        double deltaHeading = (deltaRight - deltaLeft) / WHEEL_BASE;  // Approximation of heading change

        // Compute the average change in forward movement
        double forwardMovement = (deltaLeft + deltaRight) / 2.0;

        // Calculate the lateral movement of the center wheel with correction for its offset
        double arcDistance = deltaHeading * CENTER_WHEEL_OFFSET; // Distance center odometer moves due to rotation
        double correctedLateralMovement = centerPosition - arcDistance;  // Adjust the center encoder's reading

        // Calculate the new position using trigonometry (X and Y position update)
        robotX += forwardMovement * Math.cos(previousHeading) - correctedLateralMovement * Math.sin(previousHeading);
        robotY += forwardMovement * Math.sin(previousHeading) + correctedLateralMovement * Math.cos(previousHeading);

        // Update the previous heading
        previousHeading += deltaHeading;

    }


    /**
     * Driving Method, allows for easy movement
     */
    private void driveTo(int targetX, int targetY){


        // Continue adjusting position until the robot reaches the target
        while (opModeIsActive()) {


            // Update odometry to get current position
            locate();

            double currentHeading = previousHeading;
            headingError = Range.clip((currentHeading - previousHeading) * kP, -0.4,0.4);

            // Calculate the difference in x and y
            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;



            // Calculate the distance to the target
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            double absDistance = Math.abs(distanceToTarget);
            double absHeadingError = Math.abs(headingError);

            // If the robot is within the tolerance, stop
            if (absDistance <= TOLERANCE && absHeadingError <= HEADING_TOLERANCE) {
                halt();
                break;
            }


            if(highBasket) highBasketPosition();
            if(reach) reach();

            // Calculate the angle to the target (in radians)
            double targetAngle = Math.atan2(deltaY, deltaX);


            // Calculate unit circle coordinates
            double unitX = Math.cos(targetAngle);
            double unitY = Math.sin(targetAngle);


            //Determine speed for P controller
            calculateDriveSpeed();


            // Apply mecanum wheel algorithm using unit circle coordinates
            double rightFrontPower = -headingError + unitY - unitX;
            double rightBackPower = -headingError + unitY + unitX;
            double leftFrontPower = headingError + unitY + unitX;
            double leftBackPower = headingError + unitY - unitX;


            // Set motor powers for mecanum wheels
            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);


            // Update telemetry
            telemetry.addData("X Position", robotX);
            telemetry.addData("Y Position", robotY);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Target Angle (radians)", targetAngle);
            telemetry.update();
        }
    }


    /**
     * Driving Method, has the ability to change its heading during movement
     */
    private void driveToWithHeading(int targetX, int targetY, int targetHeading){

        //Turn the target heading into radians
        targetHeading = (int) Math.toRadians(targetHeading);

        /*
          The locate() method will be called several times throughout the loop the ensure accurate localization
         */

        // Continue adjusting position and motor values until the robot reaches the target
        while(opModeIsActive()) {
            // Update odometry to get current position
            locate();

            // Calculate the difference in x and y
            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;

            //Calculate heading error and make sure the turning doesn't happen to quickly
            headingError = Range.clip(targetHeading - previousHeading, -0.4, 0.4);
            //Calculate the absolute value to make sure the program doesn't stop while the the negatives
            double absHeadingError = Math.abs(headingError);


            // Calculate the distance to the target
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            //Calculate the absolute value to make sure the program doesn't stop while in the negatives
            double absDist = Math.abs(distanceToTarget);

            //Update location
            locate();

            // If the robot is within the tolerance, stop the method
            if (absDist <= TOLERANCE && absHeadingError <= HEADING_TOLERANCE) {
                halt();
                break;
            }

            // Calculate the angle to the target (in radians) Using inverse trig functions
            double targetAngle = Math.atan2(deltaY, deltaX);

            // Calculate unit circle coordinates to imitate a game pad joystick
            double unitX = Math.cos(targetAngle);
            double unitY = Math.sin(targetAngle);

            // Calculate Speed
            calculateDriveSpeed();

            // Apply mecanum wheel algorithm using unit circle coordinates
            double rightFrontPower = -headingError + (unitY - unitX);
            double rightBackPower = -headingError + (unitY + unitX);
            double leftFrontPower = headingError + (unitY + unitX);
            double leftBackPower = headingError + (unitY - unitX);

            //Immediately update location to make sure positions are accurate
            locate();

            // Set motor powers for mecanum wheels
            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            //Check to see if the arm should move to a position and call the corresponding method
            if(highBasket) highBasketPosition(); highBasket = false;
            if(reach) reach(); reach = false;

            //Update the location again to ensure accurate positions
            locate();


            // Update telemetry for debugging
            telemetry.addData("X Position", robotX);
            telemetry.addData("Y Position", robotY);
            telemetry.addData("Distance to Target", distanceToTarget);
            telemetry.addData("Target Angle (radians)", targetAngle);
            telemetry.update();
        }
    }

    private void calculateDriveSpeed(){
        DRIVE_SPEED = Range.clip(distanceToTarget * kP, 0, 1);
    }

    private void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        fl.setPower(leftFrontPower * DRIVE_SPEED);
        fr.setPower(rightFrontPower * DRIVE_SPEED);
        bl.setPower(leftBackPower * DRIVE_SPEED);
        br.setPower(rightBackPower * DRIVE_SPEED);
    }

    private void highBasketPosition(){
        base.setTargetPosition(300);//Placeholder
        slide.setTargetPosition(1000);//Placeholder
        arm.setTargetPosition(1000);//Placeholder
        wrist.setPosition(0.7);
        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void reach(){
        base.setTargetPosition(500);//Placeholder
        slide.setTargetPosition(0);//Placeholder
        arm.setTargetPosition(300);//Placeholder
        wrist.setPosition(0);
        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void open(){
        claw.setPosition(1);
    }
    private void close(){
        claw.setPosition(0);
    }




}   // end class