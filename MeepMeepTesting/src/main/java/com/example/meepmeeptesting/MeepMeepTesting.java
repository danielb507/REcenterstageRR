package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(625);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.75)
                .setDimensions(17.8, 17.8)
                .setStartPose(new Pose2d(-60, -60, Math.toRadians(-90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -60, Math.toRadians(90)))
//-32,65 (90) //10,60, (90)



                                // TOPLEFT -- Pose2d(-35, 60, Math.toRadians(90)
                                // ID 1
                                /*
                                .back(15)
                                .strafeLeft(10)
                                .back(5)
                                .forward(5)
                                .strafeRight(10)
                                .back(35)
                                .strafeRight(90)
                                 */

                                // Id 2

                                /*
                                .back(30)
                                .forward(10)
                                .strafeLeft(15)
                                .back(28)
                                .strafeRight(90)
                                 */

                                //ID 3

                                /*
                                .back(25)
                                .turn(Math.toRadians(90))
                                .back(5)
                                .forward(20)
                                .strafeLeft(22)
                                .back(100)
                                 */

                                //TopRight -- Pose2d(35, 60, Math.toRadians(90)

                                // Id Left
                                /*
                                .back(27)
                                .turn(Math.toRadians(-90))
                                .back(5)
                                .forward(5)
                                .strafeLeft(27)
                                .forward(37)

                                 */




                                //ID Middle
                                /*
                                .back(30)
                                .forward(27)
                                .strafeRight(40)

                                 */

                                // ID Right
                                /*
                                .back(10)
                                .strafeRight(10)
                                .back(10)
                                .forward(20)
                                .strafeRight(30)

                                 */

                                //Right(MeepMeep) Parking BlueRight (Actual)


                                .lineToLinearHeading(new Pose2d(-60, 10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-15, 10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-15, -60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(60, -15, Math.toRadians(180)))






                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.999f)
                .addEntity(myBot)
                .start();
    }
}