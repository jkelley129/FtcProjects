package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Actuator Telemetry")
public class ActuatorTelemetry extends LinearOpMode{


    @Override
    public void runOpMode(){

        DcMotor fl,fr,bl,br;
        DcMotor base,slide,arm;
        DcMotorEx leftEncoder,rightEncoder,centerEncoder;
        Servo wrist, claw;

        double robotX = 0;
        double robotY = 0;

        fl = hardwareMap.get(DcMotor.class, "fl");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder = hardwareMap.get(DcMotorEx.class, "fl");

        fr = hardwareMap.get(DcMotor.class, "fr");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "fr");

        bl = hardwareMap.get(DcMotor.class, "bl");
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerEncoder = hardwareMap.get(DcMotorEx.class, "bl");

        br = hardwareMap.get(DcMotor.class, "br");
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        waitForStart();
        while(opModeIsActive()){
            double TICKS_PER_INCH = 44;


            // Read encoder values
            double leftPosition = (double) leftEncoder.getCurrentPosition() / TICKS_PER_INCH;
            double rightPosition = (double) rightEncoder.getCurrentPosition() / TICKS_PER_INCH;
            double centerPosition = (double) centerEncoder.getCurrentPosition() / TICKS_PER_INCH;

            double previousHeading = 0;
            double WHEEL_BASE = 16.0;

            // Calculate heading based on wheel positions
            double deltaLeft = leftPosition - previousHeading;
            double deltaRight = rightPosition - previousHeading;
            double deltaHeading = (deltaRight - deltaLeft) / WHEEL_BASE;  // Approximation of heading change


            // Compute the average change in movement
            double forwardMovement = (deltaLeft + deltaRight) / 2.0;  // Forward movement
            //Lateral movement is just centerPosition


            // Calculate the new position using trigonometry
            robotX += forwardMovement * Math.cos(previousHeading) - centerPosition * Math.sin(previousHeading);
            robotY += forwardMovement * Math.sin(previousHeading) + centerPosition * Math.cos(previousHeading);


            // Update the previous heading
            previousHeading += deltaHeading;



            telemetry.addData("Robot X:  ", robotX);
            telemetry.addLine();
            telemetry.addData("Robot Y:  ", robotY);
            telemetry.addLine();
            telemetry.addData("Heading:  ", previousHeading);
            telemetry.addLine();
            telemetry.addData("Base Position:  ", base.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Slide Position:  ", slide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Arm Position:  ", arm.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Wrist Position:  ", wrist.getPosition());
            telemetry.addLine();
            telemetry.addData("Claw Position:  ", claw.getPosition());
            telemetry.update();

        }
    }
}
