/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareK9bot_team10136.HORIZONTAL_ARM_HOME;
import static org.firstinspires.ftc.teamcode.HardwareK9bot_team10136.VERTICAL_ARM_HOME;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoKnockRedBall", group="Linear Opmode")
//@Disabled
public class AutoKnockRedBall extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareK9bot_team10136 robot   = new HardwareK9bot_team10136();    // Use a K9'shardware
//    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    final double    ARM_SPEED       = 0.02 ;                            // sets rate to move servo

    static final double     FORWARD_SPEED = 0.2;
    static final double     REVERSE_SPEED = -0.2;
    static final double     TURN_SPEED    = 0.5;
    static final int MOVE_JEWEL_DRIVE_DURATION = 400;

//    /** The colorSensor field will contain a reference to our color sensor hardware object */
//    NormalizedColorSensor colorSensor;
    ColorSensor colorSensor;    // Hardware Device Object
    ColorSensor Ball= null;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;


        telemetry.addData("Status", "Autonomous Initialized and Ready to run");
        telemetry.addData("Start armPosition", robot.arm.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            armPosition -= ARM_SPEED;
//
//            robot.ARM_MIN_RANGE = -0.5;
//            robot.ARM_MAX_RANGE = 1.0;
//            // Move servo to new position.
//            telemetry.addData("Start-2 armPosition", armPosition);
//            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
//            telemetry.addData("Start-3 armPosition", armPosition);

            telemetry.addData("Start armPosition", robot.arm.getPosition());
            robot.arm.setPosition(HORIZONTAL_ARM_HOME);

            // bLedOn represents the state of the LED.
            boolean bLedOn = true;

            // get a reference to our ColorSensor object.
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

            // Set the LED in the beginning
            colorSensor.enableLed(bLedOn);

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

//            Ball=hardwareMap.colorSensor.get("ball");

            // send the info back to driver station using telemetry function.
            telemetry.addData("End armPosition", robot.arm.getPosition());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addData("Ball color ", colorSensor.blue());

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            if (colorSensor.blue()>10) {
                // Send calculated power to wheels
//                robot.leftFrontDrive.setPower(FORWARD_SPEED);
//                robot.rightFrontDrive.setPower(FORWARD_SPEED);
//                robot.leftBackDrive.setPower(FORWARD_SPEED);
//                robot.rightBackDrive.setPower(FORWARD_SPEED);

                sleep(MOVE_JEWEL_DRIVE_DURATION);

                telemetry.addData("actualArm = ", robot.arm.getPosition());
                robot.arm.setPosition(VERTICAL_ARM_HOME);

                //Move up to CrypoBox
                robot.leftFrontDrive.setPower(FORWARD_SPEED);
                robot.rightFrontDrive.setPower(FORWARD_SPEED);
                robot.leftBackDrive.setPower(FORWARD_SPEED);
                robot.rightBackDrive.setPower(FORWARD_SPEED);

                sleep(MOVE_JEWEL_DRIVE_DURATION);

                //Rotate 90degrees
                robot.leftFrontDrive.setPower(FORWARD_SPEED+2);
                robot.rightFrontDrive.setPower(REVERSE_SPEED-2);
                robot.leftBackDrive.setPower(FORWARD_SPEED+2);
                robot.rightBackDrive.setPower(REVERSE_SPEED-2);

                sleep(MOVE_JEWEL_DRIVE_DURATION);

                //Stop
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightBackDrive.setPower(0);

                // Be sure the arm resets to the top/veritical position to keep it out of the way
                robot.arm.setPosition(VERTICAL_ARM_HOME);
            }

        }
    }
}
