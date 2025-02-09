package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.RockinBot;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        RockinBot r = new RockinBot();
        r.initializeHardwareVariables();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        telemetry.addData("This code was last updated", "1/6/2024, 4:30 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.setAscentStick(r.ASCENT_MIN);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get input from the joysticks
            r.axial = -gamepad1.left_stick_y;
            r.lateral = gamepad1.left_stick_x;
            r.yaw = gamepad1.right_stick_x;

            r.setWheelPower();

            if(!r.wheelClimb) {
                // Send calculated power to wheels
                r.leftFrontDrive.setPower(r.leftFrontPower);
                r.rightFrontDrive.setPower(r.rightFrontPower);
                r.leftBackDrive.setPower(r.leftBackPower);
                r.rightBackDrive.setPower(r.rightBackPower);
            }

            // Control the vertical - the rotation level of the arm
            r.verticalPosition = r.vertical.getCurrentPosition();
            r.verticalAdjustedMin = (int)(0.07*r.viperSlidePosition+r.VERTICAL_MIN);

            // Set vertical into initial climb position
            if (gamepad1.dpad_up) {
                // Hook onto the bar
                r.setVertical(r.VERTICAL_CLIMB_POSITION, 2500);
            }
            // Active climb
            else if (gamepad1.dpad_down) {
                if (r.vertical.getCurrentPosition() > 100) {
                    r.wheelClimb = true;
                    r.vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    r.vertical.setPower(-0.8);
                    r.leftBackDrive.setPower(0.5);
                    r.rightBackDrive.setPower(0.5);
                }
                else {
                    r.leftBackDrive.setPower(0);
                    r.rightBackDrive.setPower(0);
                    r.wheelClimb = false;
                    r.setVertical(r.VERTICAL_MIN, 1000);
                }
            }

            // If the right button is pressed AND it can safely raise further
            else if (gamepad1.dpad_right && r.verticalPosition < r.VERTICAL_MAX) {
                r.setVertical(Math.min(r.VERTICAL_MAX, r.verticalPosition + 50), 2000);
            }
            // If the left button is pressed AND it can safely lower without changing the viper
            else if (gamepad1.dpad_left && r.verticalPosition > r.VERTICAL_MAX_VIPER) {
                r.setVertical(Math.max(r.VERTICAL_MAX_VIPER, r.verticalPosition - 50), 1500);
            }
            // If the left button is pressed AND it can safely lower further
            else if (gamepad1.dpad_left && r.verticalPosition > r.verticalAdjustedMin) {
                if (r.viperSlidePosition > r.VIPER_MAX_WIDE) {
                    r.setViper(r.VIPER_MAX_WIDE, 1000);
                }
                r.setVertical(Math.max(r.verticalAdjustedMin, r.verticalPosition - 50),1000);
            }

            // Control the viper slide - how much it extends
            r.viperSlidePosition = r.viperSlide.getCurrentPosition();
            // If the right button is pressed AND it can safely extend further, and the viper can go all the way up
            if (gamepad1.right_trigger > 0 && r.viperSlidePosition < r.VIPER_MAX_TALL && r.verticalPosition > r.VERTICAL_MAX_VIPER) {
                r.viperSlide.setTargetPosition(r.viperSlidePosition + 200);
                ((DcMotorEx) r.viperSlide).setVelocity(gamepad1.right_trigger*4000);
                r.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // If the right button is pressed AND it can safely extend further, and the viper is under the wide limit
            else if (gamepad1.right_trigger > 0 && r.viperSlidePosition < r.VIPER_MAX_WIDE && r.verticalPosition < r.VERTICAL_MAX_VIPER) {
                r.viperSlide.setTargetPosition(r.viperSlidePosition + 200);
                ((DcMotorEx) r.viperSlide).setVelocity(gamepad1.right_trigger * 4000);
                r.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // If the right button is pressed AND it can safely retract further
            else if (gamepad1.left_trigger > 0 && r.viperSlidePosition > r.VIPER_MIN) {
                r.viperSlide.setTargetPosition(Math.max(r.VIPER_MIN, r.viperSlidePosition - 200));
                ((DcMotorEx) r.viperSlide).setVelocity(gamepad1.left_trigger*4000);
                r.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the claw
            if (gamepad1.right_bumper && r.claw_position < r.CLAW_MAX) {
                r.claw_position += 0.3;
            }
            if (gamepad1.left_bumper && r.claw_position > r.CLAW_MIN) {
                r.claw_position -= 0.15;
            }
            r.claw.setPosition(r.claw_position);

            // Y/Triangle: High basket scoring position.
            if (gamepad1.y) {
                r.setVertical(r.VERTICAL_MAX, 3000);
                r.setViper(r.VIPER_MAX_TALL, 2000);
            }

            // A/X button: Complete Retraction- Viper and vertical completely retracted and down
            if (gamepad1.a) {
                r.setViper(r.VIPER_MIN, 4000);
                r.setVertical(r.VERTICAL_MIN, 700);
            }

            // X/Square: The viper slide is completely retracted but the vertical is in submersible position.
            if (gamepad1.x) {
                r.setVertical(355, 3000);
                r.setViper(0, 1500);
                r.wheelClimb = false;
            }

            // B/Circle: The vertical is in submersible position and the viper slide is all the way out.
            if (gamepad1.b) {
                r.setVertical(380, 1800);
                r.setViper(1900, 2000);
            }

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();
        }
        r.claw.close();
    }
}