/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "Auto_1.0", group = "Sensor")

public class Auto extends LinearOpMode {

    //declaring hardware

    ColorSensor colorSensor;    // top color sensor
    OpticalDistanceSensor ODS;  // ODS for determining the beacons
    ModernRoboticsI2cGyro gyro; // gyro for aligning robot

    //declare drive motors
    DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //variables

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        int colorBeacon1 = 0;
        boolean isCloseEnough = false;
        int heading = 0;

        /* values for int color
           blue = 1
           red = 2
        */

        //initialize sensors

        colorSensor = hardwareMap.colorSensor.get("cc");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        //initialize drive motors

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");

        //reverse left motors

        motorBackLeft.setDirection((DcMotor.Direction.REVERSE));
        motorFrontLeft.setDirection((DcMotor.Direction.REVERSE));

        //Switching to passive mode as enableLED = false

        colorSensor.enableLed(false); //Don't delete this

        //calibrating the gyro sensor to learn what is 0 degree orientation

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.

        while (!isStopRequested() && gyro.isCalibrating())
        {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {

            //converting and spitting out sensor values for easy debugging

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            //getting value of orientation of robot

            heading = gyro.getHeading();

            //printing out sensor data

            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green ", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("ODS normal ", ODS.getLightDetected());
            telemetry.addData("Gyro heading ", heading);
            telemetry.update();


        }
    }

    //detecting color and changing colorBeacon1 state for the first beacon

    public int senseBeacons(int beaconColor, boolean isCloseEnough)
    {

        if(colorSensor.blue() > colorSensor.red() && isCloseEnough)
        {
            beaconColor = 2;
            telemetry.addLine("BLUE");
            telemetry.update();

        }
        else if (colorSensor.blue() < colorSensor.red() && isCloseEnough)
        {
            beaconColor =1;
            telemetry.addLine("RED");
            telemetry.update();

        }
        return beaconColor;
    }

    //determines if the robot is close enough to start detecting without error

    public void closeEnough(boolean isCloseEnough)
    {

        while(!(ODS.getLightDetected() >= 0.045))
        {
            telemetry.addLine("The robot needs to get closer!");
            telemetry.update();
            isCloseEnough = false;
            driveForward();

        }

            telemetry.addLine("Close enough to detect beacons!");
            telemetry.update();

            isCloseEnough = true;

    }
    //Straighten to whatever degree is inputted with a set threshold

    public void alignToDegree(int heading, int deg, int threshold) throws InterruptedException
    {
        boolean degreeAchieved = false;
        while((heading > deg + threshold || heading < deg - threshold) && degreeAchieved == false)
        {
            telemetry.addData("Aligning to ", deg);
            telemetry.addData("Current heading ", heading);
            telemetry.update();
            if(heading  > deg) {
                spinLeftCONT();

            } else if (heading < deg){
                spinRightCONT();

            }

        }

        degreeAchieved = true;
        telemetry.addData("Currently at a ",heading);
        telemetry.addData("Intended target was", deg);
        telemetry.update();
    }

    public void driveForward ()
    {
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
        motorFrontLeft.setPower(1);
    }

    public void driveForward(int time) throws InterruptedException
    {
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
        motorFrontLeft.setPower(1);

        Thread.sleep(time*1000);
        stopDriving();

    }

    public void stopDriving()
    {

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);

    }

    public void driveLeft(int time) throws InterruptedException
    {

        motorBackRight.setPower(-1);
        motorBackLeft.setPower(1);
        motorFrontLeft.setPower(-1);
        motorFrontRight.setPower(1);

        Thread.sleep(time * 1000);
        stopDriving();
    }

    public void driveRight(int time) throws InterruptedException
    {
        motorBackRight.setPower(1);
        motorBackLeft.setPower(-1);
        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(-1);

        Thread.sleep(time * 1000);
        stopDriving();
    }

    public void driveBackground(int time) throws InterruptedException
    {
        motorFrontRight.setPower(-1);
        motorBackRight.setPower(-1);
        motorBackLeft.setPower(-1);
        motorFrontLeft.setPower(-1);

        Thread.sleep(time * 1000);
        stopDriving();

    }

    public void spinRight(int time) throws InterruptedException
    {
        motorBackLeft.setPower(1);
        motorBackRight.setPower(-1);
        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(-1);

        Thread.sleep(time * 1000);
        stopDriving();
    }

    public void spinLeft(int time) throws InterruptedException
    {
        motorBackLeft.setPower(-1);
        motorBackRight.setPower(1);
        motorFrontLeft.setPower(-1);
        motorFrontRight.setPower(1);

        Thread.sleep(time * 1000);
        stopDriving();
    }
    public void spinRightCONT()
    {
        motorBackLeft.setPower(1);
        motorBackRight.setPower(-1);
        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(-1);


    }

    public void spinLeftCONT() {
        {
            motorBackLeft.setPower(-1);
            motorBackRight.setPower(1);
            motorFrontLeft.setPower(-1);
            motorFrontRight.setPower(1);
        }

    }
}
