package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Create a LinearOpMode variable so you can pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "01/31/2025, 11:55 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.configureOtos();

        RobotLog.vv("Rockin' Robots", "Quick Test Code");
        r.setClaw(r.CLAW_MAX);
        r.setAscentStick(r.ASCENT_MAX);
        r.setWrist(r.WRIST_DROPOFF);
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}