/*
 * File:          u3_exam_Espinoza_Angelo.c
 * Date:          July 18th, 2019
 * Description:
 * Author:        Angelo D. Espinoza Valarezo
 * Modifications:
 */

/* WEBOTS LIBRARIES */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

/* C LIBRARIES */
#include <stdio.h>
#include <stdlib.h>

/* MACROS */
#define TIME_STEP 64
#define MAX_BITS_OBJ 65535
#define MAX_BITS_ENE 255
#define RADIUS_WHEELS 0.04

#define MAX_VELOCITY 30.3687
#define VELOCITY_AUTONOMOUS 10
#define VELOCITY_MANUAL 7.5
#define DISTANCE_OBSTACLE 17
#define DISTANCE_ENEMY_LVL1 40
#define DISTANCE_ENEMY_LVL2 30
#define DISTANCE_ENEMY_LVL3 20
#define TURN_RATIO 45
#define VELOCITY_POST 3

#define PI 3.141592

/* PROTOFUNCTIONS */
float bitsToCentimeters(float centimeters);

float bitsToCentimeters2(float centimeters);

void stopAllWheels(WbDeviceTag motor_1, WbDeviceTag motor_2,
                   WbDeviceTag motor_3);

float linearVelocity(float meters_per_second);

float degreesSec2RadSec(void);

float revolutionToRadians(float radians);

void turnPost(WbDeviceTag motor_post);

void turnGun(WbDeviceTag motor_gun);

void stopPost(WbDeviceTag motor_post);

void manual(int key, WbDeviceTag motor_1, WbDeviceTag motor_2,
            WbDeviceTag motor_3);

void autonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                WbDeviceTag motor_3, WbDeviceTag motor_post, WbDeviceTag
                motor_gun, WbDeviceTag position_sensor_detector, WbDeviceTag
                position_sensor_gun, double distance_sensor_value1, double
                distance_sensor_value2, float desired_centimeters, WbDeviceTag
                distance_detector, WbDeviceTag distance_gun);

void stopRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag motor_3);

void moveForwardRobotManual(WbDeviceTag motor_1, WbDeviceTag motor_2,
                            WbDeviceTag motor_3);

void moveBackwardRobot(WbDeviceTag motor_1, WbDeviceTag motor_2,
                             WbDeviceTag motor_3);
void moveLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3);

void moveRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                  motor_3);

void turnLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3);

void turnRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3);

void moveForwardRobotAutonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                                WbDeviceTag motor_3);



/* STATES */
enum {
    AUTONOMOUS,
    MANUAL
};

/* GLOBAL VARIABLES */
int counter_left, counter_right = 0;
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int counter = 0;
char *msg1 = "Distance sensor right value:";
char *msg2 = "Position sensor wheel 1:";
char *msg3 = "Distance sensor left value:";
char *msg4 = "Position sensor wheel 2:";
char *msg5 = "Position sensor wheel 3:";
double position_sensor_detector_value;
int main(int argc, char **argv) {

   wb_robot_init();

   /* IMPORTING MOTORS */
    WbDeviceTag motor_1 = wb_robot_get_device("motor1");
    WbDeviceTag motor_2 = wb_robot_get_device("motor2");
    WbDeviceTag motor_3 = wb_robot_get_device("motor3");
    WbDeviceTag motor_post = wb_robot_get_device("motor_post");
    WbDeviceTag motor_gun = wb_robot_get_device("motor_gun");

   /* SETTING POSITION OF THE MOTORS */
    wb_motor_set_position(motor_1, INFINITY);
    wb_motor_set_position(motor_2, INFINITY);
    wb_motor_set_position(motor_3, INFINITY);
    wb_motor_set_position(motor_post, INFINITY);
    wb_motor_set_position(motor_gun, INFINITY);

   /* IMPORTING DISTANCE SENSORS */
    WbDeviceTag distance_sensor1 = wb_robot_get_device("distance_sensor1");
    WbDeviceTag distance_sensor2 = wb_robot_get_device("distance_sensor2");
    WbDeviceTag distance_detector = wb_robot_get_device("distance_detector");
    WbDeviceTag distance_gun = wb_robot_get_device("distance_gun");

   /* ENABLING ENCODERS */
    wb_distance_sensor_enable(distance_sensor1, TIME_STEP);
    wb_distance_sensor_enable(distance_sensor2, TIME_STEP);
    wb_distance_sensor_enable(distance_detector, TIME_STEP);
    wb_distance_sensor_enable(distance_gun, TIME_STEP);

   /* IMPORTING POSITION SENSOR */
    WbDeviceTag position_sensor1 = wb_robot_get_device("position_sensor1");
    WbDeviceTag position_sensor2 = wb_robot_get_device("position_sensor2");
    WbDeviceTag position_sensor3 = wb_robot_get_device("position_sensor3");
    WbDeviceTag position_sensor_detector = wb_robot_get_device(
                                           "position_sensor_detector");
    WbDeviceTag position_sensor_gun = wb_robot_get_device("position_sensor_gun");

   /* ENABLING POSITION SENSORS */
    wb_position_sensor_enable(position_sensor1, TIME_STEP);
    wb_position_sensor_enable(position_sensor2, TIME_STEP);
    wb_position_sensor_enable(position_sensor3, TIME_STEP);
    wb_position_sensor_enable(position_sensor_detector, TIME_STEP);
    wb_position_sensor_enable(position_sensor_gun, TIME_STEP);

   /* ENABLING THE KEYBOARD */
    wb_keyboard_enable(TIME_STEP);

   /* VARIABLES */
    double distance_sensor_value1;
    double distance_sensor_value2;

    double position_sensor_value1;
    double position_sensor_value2;
    double position_sensor_value3;

    float desired_centimeters = bitsToCentimeters(DISTANCE_OBSTACLE);

    int key;
    int robot_status = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        key = wb_keyboard_get_key();

        turnPost(motor_post);
        stopPost(motor_gun);

        if (key == 'W') {
            robot_status = MANUAL;
            printf("MANUAL MODE ACTIVATED\n");
        }
        else if (key == 'G') {
            robot_status = AUTONOMOUS;
            printf("AUTONOMOUS MODE ACTIVATED\n");
            // turnPost(motor_post);
        }
        else {
            stopAllWheels(motor_1, motor_2, motor_3);
        }

        distance_sensor_value1 = wb_distance_sensor_get_value(distance_sensor1);
        distance_sensor_value2 = wb_distance_sensor_get_value(distance_sensor2);

        position_sensor_value1 = wb_position_sensor_get_value(position_sensor1);
        position_sensor_value2 = wb_position_sensor_get_value(position_sensor2);
        position_sensor_value3 = wb_position_sensor_get_value(position_sensor3);



        switch (robot_status) {
            case MANUAL:     manual(key, motor_1, motor_2, motor_3);
                             break;
            case AUTONOMOUS: autonomous(motor_1, motor_2, motor_3, motor_post,
                             motor_gun, position_sensor_detector,
                             position_sensor_gun, distance_sensor_value1,
                             distance_sensor_value2, desired_centimeters,
                             distance_detector, distance_gun);
                             break;
        }



        // printf("%s  %.4f || %s %.4f || %s %.4f || %s %.4f || %s %.4f\n",
        // msg1, distance_sensor_value1, msg2, position_sensor_value1, msg3,
        // distance_sensor_value2, msg4, position_sensor_value2, msg5,
        // position_sensor_value3);
        // printf("\t\t\t\t     %s %.2fm || %s %.2fm\n",
        // msg1, distance_sensor_value1 * 0.2/MAX_BITS_OBJ, msg3,
        // distance_sensor_value2 * 0.2/MAX_BITS_OBJ);

    };

    wb_robot_cleanup();

    return 0;
}


float bitsToCentimeters(float centimeters) {
    return (MAX_BITS_OBJ*centimeters)/(20);
}

float bitsToCentimeters2(float centimeters) {
    return (MAX_BITS_ENE*centimeters/(40));
}

void stopAllWheels(WbDeviceTag motor_1, WbDeviceTag motor_2,
                   WbDeviceTag motor_3) {
     wb_motor_set_velocity(motor_1, 0);
     wb_motor_set_velocity(motor_2, 0);
     wb_motor_set_velocity(motor_3, 0);
}

void stopRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag motor_3) {
     wb_motor_set_velocity(motor_1, 0);
     wb_motor_set_velocity(motor_2, 0);
     wb_motor_set_velocity(motor_3, 0);
}

void moveForwardRobotManual(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                            motor_3) {
     wb_motor_set_velocity(motor_1,VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_2,VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_3,0);
}

void moveBackwardRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                       motor_3) {
     wb_motor_set_velocity(motor_1,-VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_2,-VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_3,0);
}

void moveLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3) {
    wb_motor_set_velocity(motor_1, 4);
    wb_motor_set_velocity(motor_2,-4);
    wb_motor_set_velocity(motor_3, 8);
}
void moveRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3) {
    wb_motor_set_velocity(motor_1,-4);
    wb_motor_set_velocity(motor_2, 4);
    wb_motor_set_velocity(motor_3,-8);
}
void turnLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3) {
    wb_motor_set_velocity(motor_1,degreesSec2RadSec());
    wb_motor_set_velocity(motor_2,-degreesSec2RadSec());
    wb_motor_set_velocity(motor_3,-degreesSec2RadSec());
}
void turnRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3) {
    wb_motor_set_velocity(motor_1,-degreesSec2RadSec());
    wb_motor_set_velocity(motor_2,degreesSec2RadSec());
    wb_motor_set_velocity(motor_3,degreesSec2RadSec());
}

void moveForwardRobotAutonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                                WbDeviceTag motor_3) {
    wb_motor_set_velocity(motor_1, VELOCITY_AUTONOMOUS);
    wb_motor_set_velocity(motor_2, VELOCITY_AUTONOMOUS);
    wb_motor_set_velocity(motor_3, 0);
}

float linearVelocity(float meters_per_second) {
    float RPM;
    float linear_velocity;

    meters_per_second = meters_per_second / RADIUS_WHEELS;
    RPM = (meters_per_second * 290) / MAX_VELOCITY;
    linear_velocity = ((2 * PI * RADIUS_WHEELS) / 60) * RPM;

    return linear_velocity;
}

float degreesSec2RadSec(void) {
    float rad_sec;

    rad_sec = TURN_RATIO * 0.017452;

    return rad_sec;
}

float revolutionToRadians(float radians) {
    int integer_part;
    float result;
    float decimal_part;
    float turns;

    integer_part = radians/(2*PI);
    decimal_part = radians/(2*PI);

    turns = decimal_part - integer_part;

    result = turns * (2*PI);

    return result;
}

void turnPost(WbDeviceTag motor_post) {
    wb_motor_set_velocity(motor_post, VELOCITY_POST);
}

void turnGun(WbDeviceTag motor_gun) {
    wb_motor_set_velocity(motor_gun, VELOCITY_POST);
}

void stopPost(WbDeviceTag motor_post) {
    wb_motor_set_velocity(motor_post, 0);
}

void manual(int key, WbDeviceTag motor_1, WbDeviceTag motor_2,
            WbDeviceTag motor_3) {
    switch (key) {
         /* MOVE FORWARD */
        case WB_KEYBOARD_UP:    moveForwardRobotManual(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.2lfm\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE BACKWARD */
        case WB_KEYBOARD_DOWN:  moveBackwardRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.2lfm\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE LEFT */
        case WB_KEYBOARD_LEFT:  moveLeftRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.2lfm\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE RIGHT */
        case WB_KEYBOARD_RIGHT: moveRightRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.2lfm\n",
                                linearVelocity(0.3));
                                break;
        /* TURN TO THE LEFT */
        case 'A':               turnLeftRobot(motor_1, motor_2, motor_3);
                                printf("Degrees/s are: %ddeg/s\n", 45);
                                break;
        /* TURN TO THE RIGHT */
        case 'S':               turnRightRobot(motor_1, motor_2, motor_3);
                                printf("Degrees/s are: %ddeg/s\n", 45);
                                break;
        default:                stopRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.2lfm\n",
                                linearVelocity(0));
                                break;
    }
}

void autonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                WbDeviceTag motor_3, WbDeviceTag motor_post, WbDeviceTag
                motor_gun, WbDeviceTag position_sensor_detector, WbDeviceTag
                position_sensor_gun, double distance_sensor_value1, double
                distance_sensor_value2, float desired_centimeters, WbDeviceTag
                distance_detector, WbDeviceTag distance_gun) {



    /* VARIABLES */


    double position_sensor_gun_value = wb_position_sensor_get_value(
                                       position_sensor_gun);
    double distance_detector_value = wb_distance_sensor_get_value
                                   (distance_detector);
    double distance_detector_gun = wb_distance_sensor_get_value(distance_gun);


    float distance_enemy1 = bitsToCentimeters2(DISTANCE_ENEMY_LVL1);
    float distance_enemy2 = bitsToCentimeters2(DISTANCE_ENEMY_LVL2);
    float distance_enemy3 = bitsToCentimeters2(DISTANCE_ENEMY_LVL3);
    float max_radians = 6.28319;
    float max_revolutions = revolutionToRadians(position_sensor_detector_value);
    float desired_value;

    // printf("The encoder has passed %lf\n", position_sensor_detector_value);
    // printf("The distance detector%lf\n", distance_detector_value);
    // printf("The distance enemy %lf\n", distance_enemy1);
    // printf("max_revolutions: %f position_sensor_gun_value: %lf\n", max_revolutions, position_sensor_gun_value);

    /* MOVE FORWARD */
    moveForwardRobotAutonomous(motor_1, motor_2, motor_3);
    // printf("Linear Velocity is: %.2lfm\n", linearVelocity(0.4));

    if (distance_detector_value < distance_enemy1) {
        flag1 = 1;
        position_sensor_detector_value = wb_position_sensor_get_value
                                        (position_sensor_detector);
    }

    if (flag1 == 1) {
        stopRobot(motor_1, motor_2, motor_3);
        // stopPost(motor_post);
        turnGun(motor_gun);
        desired_value = position_sensor_detector_value;

        if (position_sensor_gun_value >= position_sensor_detector_value) {
            stopPost(motor_gun);
            printf("THATHATHATHATHA!\n");
        }
    }

    if (distance_detector_value < distance_enemy2) {
        flag2 = 1;
        position_sensor_detector_value = wb_position_sensor_get_value
                                        (position_sensor_detector);
    }
    if (flag2 == 1) {
        stopRobot(motor_1, motor_2, motor_3);
        // stopPost(motor_post);
        turnGun(motor_gun);

        if (position_sensor_gun_value >= position_sensor_detector_value) {
            stopPost(motor_gun);
            printf("THATHATHATHATHATHATHATHATHA!\n");

        }
    }

    if (distance_detector_value < distance_enemy3) {
        flag3 = 1;
        position_sensor_detector_value = wb_position_sensor_get_value
                                        (position_sensor_detector);
    }

    if (flag3 == 1) {
        stopRobot(motor_1, motor_2, motor_3);
        // stopPost(motor_post);
        turnGun(motor_gun);

        if (position_sensor_gun_value >= position_sensor_detector_value) {
            stopPost(motor_gun);
            printf("THATHATHATHATHATHATHATHATHATHATHATHATHATHATHA!\n");


        }
    }

    if (distance_sensor_value2 <= desired_centimeters && distance_sensor_value2
        < distance_sensor_value1) {
        counter_left++;
    }

    /* AVOID OBSTACLES LEFT */
    if (counter_left >= 1 && counter_left <= 70) {
        turnRightRobot(motor_1, motor_2, motor_3);
        // printf("Degrees/s are: %ddeg/s\n", 45);
        counter_left++;
    }
    else {
        counter_left = 0;
    }

    /* AVOID OBSTACLES RIGHT */
    if (distance_sensor_value1 < desired_centimeters && distance_sensor_value1
        < distance_sensor_value2) {
        counter_right++;
    }
    if (counter_right >= 1 && counter_right <= 70) {
        turnLeftRobot(motor_1, motor_2, motor_3);
        // printf("Degrees/s are: %ddeg/s\n", 45);
        counter_right++;
    }
    else {
        counter_right = 0;
    }

}
