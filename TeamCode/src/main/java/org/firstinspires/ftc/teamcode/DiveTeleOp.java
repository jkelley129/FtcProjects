package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DiveTeleOp")
public class DiveTeleOp extends LinearOpMode {



    @Override
    public void runOpMode() {
        DcMotor fl,fr,bl,br;
        DcMotor base,slide,arm;
        Servo wrist, claw;

        final double kP = 0.0135; //Tune

        // Initialize the motor and set the encoder mode
        fl = hardwareMap.get(DcMotor.class, "fl");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr = hardwareMap.get(DcMotor.class, "fr");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl = hardwareMap.get(DcMotor.class, "bl");
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        double baseError;
        double baseActuatorSpeed;

        double slideError;
        double slideActuatorSpeed;

        double armError;
        double armActuatorSpeed;

        int baseTargetPosition = 0;
        int slideTargetPosition = 0;
        int armTargetPosition = 0;

        boolean actuatorMovement = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double speed = 0.7;
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Power: ", fl.getPower());
            telemetry.update();

            if(actuatorMovement){
                baseError = baseTargetPosition - base.getCurrentPosition();
                slideError = slideTargetPosition - slide.getCurrentPosition();
                armError = armTargetPosition - arm.getCurrentPosition();

                baseActuatorSpeed = Range.clip(baseError * kP, -1, 1);
                slideActuatorSpeed = Range.clip(slideError * kP, -1, 1);
                armActuatorSpeed = Range.clip(armError * kP, -1, 1);

                base.setPower(baseActuatorSpeed);
                slide.setPower(slideActuatorSpeed);
                arm.setPower(armActuatorSpeed);

                base.setTargetPosition(baseTargetPosition);
                slide.setTargetPosition(slideTargetPosition);
                arm.setTargetPosition(armTargetPosition);

                base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //Speed Changing
            if(gamepad1.a){
                speed = 0.3;
            }if(gamepad1.b){
                speed = 0.5;
            }if(gamepad1.x){
                speed = 0.7;
            }if(gamepad1.y){
                speed = 1.0;
            }

            //Driving Logic
            float vertical = gamepad1.left_stick_y;
            float horizontal = gamepad1.left_stick_x;
            float pivot = gamepad1.right_stick_x;
            fl.setPower(speed *(pivot + vertical + horizontal));
            fr.setPower(speed * (-pivot + vertical - horizontal));
            bl.setPower(speed *(pivot + vertical - horizontal));
            br.setPower(speed * (-pivot + vertical + horizontal));

            if(gamepad2.a){
                baseTargetPosition = 0;
                slideTargetPosition = 0;
                armTargetPosition = 0;
                actuatorMovement = true;
                wrist.setPosition(0.5);//Placeholder


            }if(gamepad2.b){
                baseTargetPosition = 500;// PlaceHolder
                slideTargetPosition = 0;//Correct Value
                armTargetPosition = 300;//Placeholder
                actuatorMovement = true;
                wrist.setPosition(0);//Placeholder
            }

            if(gamepad2.x){
                baseTargetPosition = 300;// Placeholder
                slideTargetPosition = 0;//Correct Value
                armTargetPosition = 1000;//Placeholder
                actuatorMovement = true;
                wrist.setPosition(0.7);//Placeholder
            }

            if(gamepad2.y){
                baseTargetPosition = 300;//Placeholder
                slideTargetPosition = 1000;//Placeholder
                armTargetPosition = 1000;//Placeholder
                actuatorMovement = true;
                wrist.setPosition(0.7);
            }

            if(gamepad2.dpad_right){
                baseTargetPosition = 0;//Placeholder
                slideTargetPosition = 1000;//Placeholder
                armTargetPosition = 1250;//Placeholder
                actuatorMovement = true;
                wrist.setPosition(0);//Placeholder
            }

            if(gamepad2.dpad_up){
                baseTargetPosition = 0;//Placeholder
                slideTargetPosition = 1000;//Placeholder
                armTargetPosition = 1850;//Placeholder
                actuatorMovement = true;
                wrist.setPosition(1);//Placeholder

            }

            if(gamepad2.dpad_down){
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-0.3);
                sleep(300);
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad2.right_bumper){
                claw.setPosition(1);
            }if(gamepad2.left_bumper){
                claw.setPosition(0);
            }


        }





    }
}