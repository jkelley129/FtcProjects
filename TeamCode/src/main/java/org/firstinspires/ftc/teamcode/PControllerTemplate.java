package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="P Controller Demo")
public class PControllerTemplate extends LinearOpMode {

    private DcMotor motor;
    private double targetPosition = 300; // Initial target position (encoder ticks)
    private double kP = 0.001; // P coefficient
    private boolean toggleTargetPosition = false;

    @Override
    public void runOpMode() {
        // Initialize the motor and set the encoder mode
        motor = hardwareMap.get(DcMotor.class, "leftFront");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Calculate the P component
            double error = targetPosition - motor.getCurrentPosition();
            double output = kP * error;

            // Set motor power with the calculated P output
            motor.setPower(Range.clip(output, -1, 1));

            // Tuning P coefficient using gamepad
            if (gamepad1.dpad_up) kP += 0.005;
            if (gamepad1.dpad_down) kP -= 0.005;

            // Toggle target position between 0 and 300 using the B button
            if (gamepad1.b && !toggleTargetPosition) {
                targetPosition = (targetPosition == 0) ? 300 : 0;
                toggleTargetPosition = true;
            } else if (!gamepad1.b) {
                toggleTargetPosition = false;
            }

            // Display P coefficient, current position, and target position
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.addData("kP", kP);
            telemetry.update();
        }
    }
}