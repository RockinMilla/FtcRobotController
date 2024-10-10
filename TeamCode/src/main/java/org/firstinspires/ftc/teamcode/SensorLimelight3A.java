/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class SensorLimelight3A extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private final ElapsedTime runtime = new ElapsedTime();

    static final double     DEFAULT_SPEED = 0.2;
    private double previousxLocation = 0;
    private double currentXLocation = 0;
    private double xDestination = 0.0;
    private double distanceToDrive = 0.0;
    private double currentLocationCheck = 0;
    private double previousLocationCheck = 0;
    private double currentXSpeed = 0;

    private double maxTurnSpeed = 0.6;
    private double currentTurnSpeed = 0;
    private double newTurnSpeed = 0;
    private double turnDirection = 0;
    private double degreesToTarget = 0;

    private double currentLimeYaw = 0;

    private Limelight3A limelight;
    private LLStatus status;
    private LLResult result;
    private double targetingLatency = 0;
    private int aprilTag = 0;

    private IMU imu = null;
    private double currentImuYaw = 0;

    double biggestXLoc = -10;
    double smallestXLoc = 10;

    //The limelight has a heading range of 62 degrees and can see the AprilTag from 10 feet away. It can be positioned at a maximum of 14 1/2 inches tall.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Define all the hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            CheckCamera();

            if(currentXLocation < smallestXLoc)
                smallestXLoc = currentXLocation;
            if(currentXLocation > biggestXLoc)
                biggestXLoc = currentXLocation;
            RobotLog.vv("RockinRobots", "Camera: smallestXLoc: %.2f, biggestXLoc: %.2f", smallestXLoc, biggestXLoc);

            if (gamepad1.dpad_up) {
                xDestination = 0.8;
            }
            else if (gamepad1.dpad_right) {
                xDestination = 1.0;
            }
            else if (gamepad1.dpad_left) {
                xDestination = 1.2;
            }
            else if (gamepad1.dpad_down) {
                xDestination = 1.4;
            }
            //DriveToX(xDestination);
            //TurnToYaw(yaw);
            LogData();
        }
        limelight.stop();
    }

    private void LogData() {
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());   //Tells us the temperatures, CPU and FPS
        telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());              //Tells us pipeline value
        telemetry.addData("Total Latency", "%.0f", targetingLatency);
        telemetry.addData("AprilTag visible", "%b", result.isValid());
        telemetry.addData("AprilTag ID", "%d", aprilTag);             // AprilTag ID
        telemetry.addData("currentXLocation", "%.1f", currentXLocation);            // Get X-Location from the robot
        //RobotLog.vv("RockinRobots", "currentXLocation: %.2f", currentXLocation);
        telemetry.addData("Driving to xLoc", "%.1f", xDestination);
        telemetry.addData("Distance to Drive", "%.1f", distanceToDrive);
        telemetry.addData("currentLimeYaw", "%.1f", currentLimeYaw);
        telemetry.addData("currentImuYaw", "%.1f", currentImuYaw);
        //RobotLog.vv("RockinRobots", "currentLimeYaw: %.1f, currentImuYaw: %.1f", currentLimeYaw, currentImuYaw);

        telemetry.update();
    }

    private void DriveToX(double xLoc) {
        if(CheckCamera()) {
            distanceToDrive = currentXLocation - xLoc;
            //RobotLog.vv("RockinRobots", "Camera: currentXLocation: %.2f, distanceToDrive: %.2f, currentXSpeed: %.2f", currentXLocation, distanceToDrive, currentXSpeed);
        } else {
            double calculatedXLocation = currentXLocation + currentXSpeed*(runtime.seconds() - currentLocationCheck);
            distanceToDrive = calculatedXLocation - xLoc;
            //RobotLog.vv("RockinRobots", "Veloci: calculaXLocation: %.2f, distanceToDrive: %.2f", calculatedXLocation, distanceToDrive);
        }

        if(Math.abs(distanceToDrive) > 0.15) {
            double direction = Math.signum(distanceToDrive);
            leftFrontDrive.setPower(DEFAULT_SPEED * -direction);
            rightFrontDrive.setPower(DEFAULT_SPEED * direction);
            leftBackDrive.setPower(DEFAULT_SPEED * direction);
            rightBackDrive.setPower(DEFAULT_SPEED * -direction);
        }
        else {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }

    private boolean CheckCamera() {
        status = limelight.getStatus();
        result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            targetingLatency = result.getTargetingLatency();


            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                // Access AprilTag ID results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    aprilTag = fr.getFiducialId();
                }
                currentXLocation = botpose.getPosition().x;
                //RobotLog.vv("RockinRobots", "currentXLocation: %.2f", currentXLocation);
                currentLocationCheck = runtime.seconds();
                if(currentLocationCheck - .5 > previousLocationCheck) {
                    currentXSpeed = (previousxLocation - currentXLocation) / (previousLocationCheck - currentLocationCheck);
                    previousLocationCheck = currentLocationCheck;
                    previousxLocation = currentXLocation;
                }
                //telemetry.addData("Y-Location", "%.1f", botpose.getPosition().y);            // Get Y-Location from the robot
                //telemetry.addData("Heading", "%.0f", botpose.getOrientation().getYaw());     // Get Heading from the robot
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    private double currentXSpeed() {
        double startingXLocation = currentXLocation;
        double startingTime = runtime.seconds();
        sleep(1000);
        CheckCamera();
        double endingXLocation = currentXLocation;
        double endingTime = runtime.seconds();
        double currentXSpeed = (endingXLocation - startingXLocation) / (endingTime - startingTime);
        //RobotLog.vv("RockinRobots", "currentXSpeed: %.2f", currentXSpeed);
        return currentXSpeed;
    }

    private void turnToYaw(double targetYaw) {
        currentImuYaw = imu.getRobotYawPitchRollAngles().getYaw();
        degreesToTarget = currentImuYaw - targetYaw;
        turnDirection = Math.signum(degreesToTarget);

        if (Math.abs(degreesToTarget) < 2) {
            stopMoving();
        }
        else {
            newTurnSpeed = Math.abs(degreesToTarget)/200;
            newTurnSpeed = Math.min(newTurnSpeed, currentTurnSpeed + .1);
            newTurnSpeed = Math.max(newTurnSpeed, .2);
            currentTurnSpeed = newTurnSpeed;
        }

        leftFrontDrive.setPower(currentTurnSpeed*turnDirection);
        rightFrontDrive.setPower(currentTurnSpeed*-turnDirection);
        leftBackDrive.setPower(currentTurnSpeed*turnDirection);
        rightBackDrive.setPower(currentTurnSpeed*-turnDirection);
    }

    private void stopMoving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}