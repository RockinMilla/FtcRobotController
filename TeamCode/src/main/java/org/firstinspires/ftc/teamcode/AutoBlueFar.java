package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Far", group="Robot")
public class AutoBlueFar extends LinearOpMode {
    private double launcherVelocity = 1140; // Change this when changing launcher velocity
    private double defaultIntakePower = 1; // Change this when changing pickup intake power

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "5/18/2026, 3:54 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.intakePower(defaultIntakePower);
        r.launcherVelocity(launcherVelocity);

        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos( -100, 800, 95, 10, 2, 2);
        r.driveToPos(-1250, 800, 95, 10, 2, 2, true);
        sleep(400);
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(-1250,   200, 100, 10, 2, 2, true);
        sleep(300);
        shootBalls(r);

        // 4th set of balls
        r.driveToPos(-1250,   200, 100, 10, 2, 2, true);
        sleep(300);
        shootBalls(r);

        // Park
        r.driveToPos(-500, 200, 70); 
        r.launcherVelocity(0);
        r.intakePower(0);
        r.lightsOff();
    }

    private void shootBalls(RockinBot r) {
        r.driveToPos(10, 250, 30, 10, 1.5, 2);
        r.waitForLaunchers(launcherVelocity);
        sleep(500);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}