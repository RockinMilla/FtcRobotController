package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime(); // TODO: add a telemetry comment to the screen that tells us how long the auto program took to execute.

    @Override
    public void runOpMode() {
        // Create a LinearOpMode variable so you can pass it to the RockinBot constructor // TODO: make this a better comment or delete it
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        // Send telemetry message to signify robot waiting; // TODO make this comment better
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "01/31/2025, 11:55 am"); // Todo: Update this date when the code is updated
                                                                                // TODO: We should create a different marker to find these code update spots
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.configureOtos();

        RobotLog.vv("Rockin' Robots", "Quick Test Code");
        // TODO: Your next goal is to prove that you can control all three servos from the AutoDrive program
        // TODO: try going both directions so you can find the failure without rebooting the robot
        r.setClaw(r.CLAW_MAX);
        r.setAscentStick(r.ASCENT_MAX);
        r.setWrist(r.WRIST_DROPOFF);
        // TODO: You might need to add a wait to give the servos time to finish the motion
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}