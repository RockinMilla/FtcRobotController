package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Auto Drive", group="Robot")
public class CallieMoveItAutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        GoBildaPinpointDriver odo; //Maybe in the wrong spot?

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        // Send a telemetry message to signify that the robot is ready to run;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "07/28/2025, 2:46 pm"); // Todo: Update this date when the code is updated
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {
            odo.update();
            PinpointLocalizer odometry = null;
            Pose2D pos = odo.getPosition();
            // The following is commented out because it causes errors that we don't want to deal with at the moment
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            RobotLog.vv("Rockin' Robots", "Position: " + data);
            RobotLog.vv("Rockin' Robots", "Callie move it");
            sleep(10 * 1000);
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            RobotLog.vv("Rockin' Robots", "Position after Callie moved it: " + data);
            // r.driveToLoc(0, 3, 0);
            sleep(20 * 1000);

            RobotLog.vv("Rockin' Robots", "Test Done");
            telemetry.addData("Autonomous lasted", runtime.toString());
        }
    }
}