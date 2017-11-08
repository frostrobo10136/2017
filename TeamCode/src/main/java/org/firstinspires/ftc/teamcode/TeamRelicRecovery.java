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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RelicRecovery10136", group="Team10136")
//@Disabled
public class TeamRelicRecovery extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareK9bot_team10136 robot        = new HardwareK9bot_team10136();  // Use a K9's hardware
    double          leftclawPosition     = robot.LEFT_CLAW_HOME;           // Servo safe position
    double          rightclawPosition    = robot.RIGHT_CLAW_HOME;          // Servo safe position
    final double    CLAW_SPEED           = 0.05 ;                          // sets rate to move servo
    final double    LIFT_MOTOR_SPEED     = 0.15;                            // sets rate to move servo

//    double left;
//    double right;
//    double leftSide;
//    double rightSide;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello RoboFalcon Driver");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double leftSide;
        double rightSide;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        leftSide = gamepad1.right_stick_x;   //Capture left side x
        rightSide = gamepad1.right_stick_x;  //Capture right side x

        robot.leftFrontDrive.setPower(left);
        robot.rightFrontDrive.setPower(right);
        robot.leftBackDrive.setPower(left);
        robot.rightBackDrive.setPower(right);

        // Use gamepad Y & A raise and lower the arm
        //if (gamepad1.a)
        //    armPosition += ARM_SPEED;
        //else if (gamepad1.y)
        //    armPosition -= ARM_SPEED;

        // Use gamepad X & B to open and close the claw
        if (gamepad2.x) {
            leftclawPosition += CLAW_SPEED;
            rightclawPosition -= CLAW_SPEED;
            leftclawPosition = Range.clip(leftclawPosition, robot.LEFT_CLAW_MIN_RANGE, robot.LEFT_CLAW_MAX_RANGE);
            rightclawPosition = Range.clip(rightclawPosition, robot.RIGHT_CLAW_MAX_RANGE, robot.RIGHT_CLAW_MIN_RANGE);

            robot.leftClaw.setPosition(leftclawPosition);
            robot.rightClaw.setPosition(rightclawPosition);
        }
        else if (gamepad2.b) {
            leftclawPosition -= CLAW_SPEED;
            rightclawPosition += CLAW_SPEED;
            leftclawPosition = Range.clip(leftclawPosition, robot.LEFT_CLAW_MIN_RANGE, robot.LEFT_CLAW_MAX_RANGE);
            rightclawPosition = Range.clip(rightclawPosition, robot.RIGHT_CLAW_MAX_RANGE, robot.RIGHT_CLAW_MIN_RANGE);

            robot.leftClaw.setPosition(leftclawPosition);
            robot.rightClaw.setPosition(rightclawPosition);
        }
        else if (gamepad2.y) {
            telemetry.addData("gamepad2.a", gamepad2.y);
            robot.liftMotor.setPower(LIFT_MOTOR_SPEED);
            //sleep(40);
            //robot.liftMotor.setPower(0);
        }
        else if (gamepad1.a) {
            telemetry.addData("gamepad2.a", gamepad2.a);
            robot.liftMotor.setPower(-LIFT_MOTOR_SPEED);
            //sleep(40);
            //robot.liftMotor.setPower(0);
        }
        else if (gamepad1.dpad_left) {
            telemetry.addData("dpad_left",  gamepad1.dpad_left);
            robot.leftFrontDrive.setPower(-2.0);
            robot.rightFrontDrive.setPower(-2.0);
            robot.leftBackDrive.setPower(2.0);
            robot.rightBackDrive.setPower(2.0);
        }
        else if (gamepad1.dpad_right) {
            telemetry.addData("dpad_right", gamepad1.dpad_right);
            robot.leftFrontDrive.setPower(2.0);
            robot.rightFrontDrive.setPower(2.0);
            robot.leftBackDrive.setPower(-2.0);
            robot.rightBackDrive.setPower(-2.0);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("after-leftclawPosition",  "%.2f", leftclawPosition);
        telemetry.addData("after-rightclawPosition",  "%.2f", rightclawPosition);
        telemetry.addData("leftMotorPower",  "%.2f", left);
        telemetry.addData("rightMotorPower", "%.2f", right);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
