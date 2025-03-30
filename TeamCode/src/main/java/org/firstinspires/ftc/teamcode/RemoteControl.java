package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {
        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        RockinBot r = new RockinBot();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        telemetry.addData("This code was last updated", "3/16/2024, 4:30 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.setAscentStick(r.ASCENT_MIN);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r.updateVertical();
            r.updateViper();

            r.setWheelPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Control the vertical - the rotation level of the arm
            r.verticalAdjustedMin = (int)(0.07*r.getViper()+r.VERTICAL_MIN);

            // Set vertical into initial climb position
            if (gamepad1.dpad_up) {
                // Hook onto the bar
                r.setVertical(r.VERTICAL_CLIMB_POSITION, 2500);
            }

            // Active climb
            else if (gamepad1.dpad_down) {
                r.activeClimb();
            }

            // If the right button is pressed AND it can safely raise further
            else if (gamepad1.dpad_right && r.getVertical() < r.VERTICAL_MAX) {
                r.setVertical(Math.min(r.VERTICAL_MAX, (int)r.getVertical() + 50), 2000);
            }
            // If the left button is pressed AND it can safely lower without changing the viper
            else if (gamepad1.dpad_left && r.getVertical() > r.VERTICAL_MAX_VIPER) {
                r.setVertical(Math.max(r.VERTICAL_MAX_VIPER, (int)r.getVertical() - 50), 1500);
            }
            // If the left button is pressed AND it can safely lower further
            else if (gamepad1.dpad_left && r.getVertical() > r.verticalAdjustedMin) {
                if (r.getViper() > r.VIPER_MAX_WIDE) {
                    r.setViper(r.VIPER_MAX_WIDE, 1000);
                }
                r.setVertical((int)Math.max(r.verticalAdjustedMin, r.getVertical() - 50),1000);
            }

            // If the right button is pressed AND it can safely extend further, and the viper can go all the way up
            if (gamepad1.right_trigger > 0 && r.getViper() < r.VIPER_MAX_TALL && r.getVertical() > r.VERTICAL_MAX_VIPER) {
                r.setViper((int)r.getViper() + 200, (int)gamepad1.right_trigger*4000);
            }
            // If the right button is pressed AND it can safely extend further, and the viper is under the wide limit
            else if (gamepad1.right_trigger > 0 && r.getViper() < r.VIPER_MAX_WIDE && r.getVertical() < r.VERTICAL_MAX_VIPER) {
                r.setViper((int)r.getViper() + 200, (int)gamepad1.right_trigger*4000);
            }
            // If the right button is pressed AND it can safely retract further
            else if (gamepad1.left_trigger > 0 && r.getViper() > r.VIPER_MIN) {
                r.setViper((int)r.getViper() - 200, (int)gamepad1.left_trigger*4000);
            }

            // Control the claw
            if (gamepad1.right_bumper && r.getClaw() < r.CLAW_MAX) {
                r.setClaw(r.getClaw() + 0.3);
            }
            if (gamepad1.left_bumper && r.getClaw() > r.CLAW_MIN) {
                r.setClaw(r.getClaw() - 0.15);
            }

            // Y/Triangle: High basket scoring position.
            if (gamepad1.y) {
                // Create a new thread for the scoring position
                Thread bgThreadTScore = new Thread(r::tScoringPosition);
                bgThreadTScore.start();
            }

            // A/X button: Complete Retraction- Viper and vertical completely retracted and down
            if (gamepad1.a) {
                // Create a new thread for the closed position
                Thread bgThreadXClosed = new Thread(r::xClosedPosition);
                bgThreadXClosed.start();
            }

            // X/Square: The viper slide is completely retracted but the vertical is in submersible position.
            if (gamepad1.x) {
                // Create a new thread for the driving position
                Thread bgThreadSDrive = new Thread(r::sDrivingPosition);
                bgThreadSDrive.start();
            }

            // B/Circle: The vertical is in submersible position and the viper slide is all the way out.
            if (gamepad1.b) {
                // Create a new thread for the driving position
                Thread bgThreadOPickup = new Thread(r::oPickupPosition);
                bgThreadOPickup.start();
            }

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();
        }
        r.disableClaw();
    }
}