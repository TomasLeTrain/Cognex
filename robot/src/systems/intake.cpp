#include "apis.h"
//
#include "auton_globals.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/rtos.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include <map>
#include <mutex>
#include <optional>

namespace intake {
// any needed variables can be specified here
// NOTE: it is recommended these variables are not changed directly by
// autos/other subsystems you should ideally provide functions to interact with
// these variables
bool is_driver = false;
bool tasks_active = false;

bool skills_middle_scoring = false;

void setSkillsMiddleScoring(bool enabled) {
    skills_middle_scoring = enabled;
}

// antijam variables
Time curr_bottom_motor_unjam_time = 0_sec;
Time curr_top_motor_unjam_time = 0_sec;

Time last_bottom_motor_unjam_time = 0_sec;
Time last_top_motor_unjam_time = 0_sec;

intake_state_t intake_state = intake_disabled;

std::map<intake_state_t, int> bottom_motor_speeds = {
    // only different one
    { slow_scoring_bottom,                  -60  },
    { scoring_bottom,                       -100 },

    // { slow_scoring_middle,         40   },
    { scoring_middle_bottom_balls,          90   },
    { scoring_middle_bottom_balls_slow,     70   },

    { scoring_middle_top_balls,             40   },
    { scoring_middle_first_ball,            127  },
    { scoring_middle_top_balls_skills,      40   },
    { scoring_middle_top_balls_skills_fast, 80   },
    { scoring_middle_top_balls_skills_slow, 30   },
    // acts as only top balls, good for driver
    { scoring_middle,                       40   },

    { scoring_long,                         127  },
    { scoring_long_top_balls,               0    },
    { scoring_long_top_balls_outake_bottom, -127 },

    { intake_disabled_open_middle,          0    },

    { intake,                               127  },
    { intake_bottom_balls,                  127  },
    { intake_bottom_top_backwards,          127  },
    { intake_top_balls,                     0    },
    { outtake,                              -127 },
    { outtake_open_middle,                  -127 },

    { score_bottom_bottom_balls,            -127 },
    { score_bottom_bottom_balls_slow,       -70  },
    { score_bottom_slow,                    -70  },

    { outtake_bottom_balls,                 -127 },
    { outtake_bottom_balls_open_middle,     -127 },

    { unjam,                                -127 },
    { outtake_very_slow,                    -40  },
};

std::map<intake_state_t, int> top_motor_speeds = {
    { slow_scoring_bottom,                  -100 },
    { scoring_bottom,                       -127 },

    // { slow_scoring_middle, 127  },
    { scoring_middle_bottom_balls,          30   },
    { scoring_middle_bottom_balls_slow,     20   },

    { scoring_middle_top_balls,             -127 },
    { scoring_middle_first_ball,            0    },
    { scoring_middle_top_balls_skills,      -100 },
    { scoring_middle_top_balls_skills_fast, -100 },
    { scoring_middle_top_balls_skills_slow, -100 },
    // acts as only top balls, good for driver
    { scoring_middle,                       -127 },

    { scoring_long,                         127  },
    { scoring_long_top_balls,               127  },
    { scoring_long_top_balls_outake_bottom, 127  },

    { score_bottom_bottom_balls,            0    },
    { score_bottom_bottom_balls_slow,       0    },
    { score_bottom_slow,                    -127 },

    { intake,                               127  },
    { intake_bottom_balls,                  0    },
    { intake_bottom_top_backwards,          -127 },

    { intake_disabled_open_middle,          0    },
    { outtake_bottom_balls,                 0    },
    { outtake_bottom_balls_open_middle,     0    },

    { outtake,                              -127 },
    { outtake_open_middle,                  -127 },

    { unjam,                                -127 },
    { outtake_very_slow,                    0    },
};

// speeds of the motors - can be positive or negative
int bottom_speed;
int top_speed;

std::optional<alliance_t> last_middle_detected_color;

std::optional<alliance_t> middle_detected_color;
std::optional<alliance_t> bottom_detected_color;

pros::Mutex intake_mutex;

bool colorSortEnabled = true;
bool driverColorSortEnabled = true;

std::optional<Time> middle_active;
std::optional<Time> long_active;
Time last_ball_time;

bool color_sort_one = false;

// intake piston stuff
intake_piston_state_t top_intake_piston_state;
intake_piston_state_t middle_intake_piston_state;

// updates state as well as piston
void setTopIntakePistonState(intake_piston_state_t intake_piston_state) {
    top_intake_piston_state = intake_piston_state;

    // top piston in allows passthrough when actuated
    top_intake_piston.set_value(top_intake_piston_state == blocking);
}

void setMiddleIntakePistonState(intake_piston_state_t intake_piston_state) {
    middle_intake_piston_state = intake_piston_state;

    // bottom piston in allows passthrough when not actuated
    middle_intake_piston.set_value(middle_intake_piston_state == blocking);
}

std::optional<alliance_t> getMiddleDetectedColor() {
    return middle_detected_color;
}

std::optional<alliance_t> getBottomDetectedColor() {
    return bottom_detected_color;
}

void set(intake_state_t new_intake_state) {
    intake_state = new_intake_state;

    if (intake_state == scoring_middle) {
        if (!middle_active) middle_active = now();
    } else {
        middle_active = std::nullopt;
    }

    if (intake_state == scoring_long) {
        if (!long_active) long_active = now();
    } else {
        long_active = std::nullopt;
    }
}

void setColorSortEnabled(bool enabled) {
    colorSortEnabled = enabled;
}

void setDriverColorSortEnabled(bool enabled) {
    driverColorSortEnabled = enabled;
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    bool intake = controller.get_digital(controls::L1);

    bool scoreBottomHeight = controller.get_digital(controls::L2);
    bool scoreMiddleHeight = controller.get_digital(controls::R2);
    bool scoreLong = controller.get_digital(controls::R1);

    bool killColorSort = controller.get_digital_new_press(controls::LEFT);

    bool unjam = controller.get_digital(controls::X);

    // one time kill switch
    if (driverColorSortEnabled == true && killColorSort) {
        driverColorSortEnabled = false;
    }

    else if (unjam)
        set(intake_state_t::unjam);

    else if (intake)
        set(intake_state_t::intake);

    else if (scoreMiddleHeight) {
        set(intake_state_t::scoring_middle);
    }

    else if (scoreBottomHeight) {
        set(intake_state_t::scoring_bottom);
    }

    else if (scoreLong) {
        set(intake_state_t::scoring_long);
    } else {
        set(intake_state_t::intake_disabled);
    }
}

std::optional<alliance_t> colorDetected(pros::Optical& sensor) {
    // detect color from color sensor
    std::optional<alliance_t> result = std::nullopt;

    // just say no balls are being detected
    if (!sensor.is_installed()) return result;

    double color_sensor_hue = sensor.get_hue();

    // intake senses something
    if (sensor.get_proximity() > 200) {
        if (color_sensor_hue > 300 || color_sensor_hue < 80)
            result = alliance_t::red;
        else if (color_sensor_hue > 140 && color_sensor_hue < 260)
            result = alliance_t::blue;
    }

    return result;
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

bool motorJammed(pros::Motor& motor) {
    double thresh_vel = 1;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

bool motorSlowed(pros::Motor& motor) {
    double thresh_vel = 100;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

// should be run in a task
void antiJam() {
    // wait for stuff to be available
    std::lock_guard lock(intake_mutex);

    Time bottom_motor_timeout = 100_msec;
    Time top_motor_timeout = 100_msec;

    // prevents constant antijam by having it only apply every 500 msec
    Time repetitive_bottom_motor_timeout = 500_msec;

    // prevents antijam while hood goes up
    // Time initial_top_motor_timeout = 200_msec;
    Time initial_top_motor_timeout = 200_msec;

    bool top_jammed = motorJammed(top_motor);
    bool bottom_jammed = motorJammed(bottom_motor);

    if (top_jammed) {
        last_top_motor_unjam_time = curr_top_motor_unjam_time;
        curr_top_motor_unjam_time = now();
    }

    if (bottom_jammed &&
        // only updates if we have waited long enough since last
        timeoutDone(repetitive_bottom_motor_timeout,
                    last_bottom_motor_unjam_time)) {
        last_bottom_motor_unjam_time = curr_bottom_motor_unjam_time;
        curr_bottom_motor_unjam_time = now();
    }

    // std::lock_guard lock(intake_mutex);

    while (true) {
        // here either top or bottom should be jammed
        int curr_bottom_speed = bottom_motor_speeds[intake_state];
        int curr_top_speed = top_motor_speeds[intake_state];

        // true if we are not unjamming anymore
        bool unjaming_bottom_done =
          // either we are not moving the motor at all
          curr_bottom_speed == 0 ||
          // or we are done antijamming
          timeoutDone(bottom_motor_timeout, last_bottom_motor_unjam_time);

        bool unjaming_top_done =
          // either we are not in the correct intake state
          // (intake_state != scoring_long) ||

          // not active if intaking
          (intake_state == intake || intake_state == intake_bottom_balls ||
           intake_state == intake_bottom_top_backwards ||
           intake_state == intake_top_balls) ||

          // or we have not finished initial timeout
          (!timeoutDone(initial_top_motor_timeout, *long_active)) ||
          // or we are done antijamming
          timeoutDone(top_motor_timeout, last_top_motor_unjam_time);

        // if both are done we have nothing left to do
        if (unjaming_bottom_done && unjaming_top_done) {
            break;
        }

        // antijamming top
        if (top_jammed) curr_top_speed = units::sgn(top_speed) * -127;

        // antijamming bottom
        if (bottom_jammed) curr_bottom_speed = 0;

        top_motor.move(curr_top_speed);
        bottom_motor.move(curr_bottom_speed);

        pros::delay(10);
    }
}

bool waitUntilBottomColor(std::optional<alliance_t> color, Time timeout) {
    Time start_time = blazing::now();

    bool triggered_timeout, detected_ball;

    while (true) {
        triggered_timeout = blazing::timeoutDone(timeout, start_time);
        detected_ball =
          color
            .transform([](alliance_t color) -> bool {
                // detected wanted color
                return color == bottom_detected_color;
            })
            // color has no value, detect ball if middle color has value
            .value_or(bottom_detected_color.has_value());

        if (triggered_timeout || detected_ball) break;
        pros::delay(10);
    }
    if (detected_ball) return true;
    return false;
}

bool waitUntilMiddleColor(std::optional<alliance_t> color, Time timeout) {
    Time start_time = blazing::now();

    bool triggered_timeout, detected_ball;

    while (true) {
        triggered_timeout = blazing::timeoutDone(timeout, start_time);
        detected_ball =
          color
            .transform([](alliance_t color) -> bool {
                // detected wanted color
                return color == middle_detected_color;
            })
            // color has no value, detect ball if middle color has value
            .value_or(middle_detected_color.has_value());

        if (triggered_timeout || detected_ball) break;
        pros::delay(10);
    }
    if (detected_ball) return true;
    return false;
}

void throwOutDetectedBall(std::optional<alliance_t> color, Time timeout) {
    bool detected_ball = waitUntilMiddleColor(color, timeout);

    if (detected_ball) {
        color_sort_one = true;
    }
}

// should be run in a task
void colorSort() {
    bool autoColorSortDisabled = !is_driver && !colorSortEnabled;
    bool driverColorSortDisabled = is_driver && !driverColorSortEnabled;

    // wait for stuff to be available
    // does not change intake, should not depend on mutex
    if (autoColorSortDisabled || driverColorSortDisabled ||
        // we don't know our alliance so we cannot color sort
        auto_alliance == alliance_t::unset) {
        pros::delay(10);
        return;
    }

    // a ball is being measured
    bool middle_wrong_color_detected = middle_detected_color
                                         .transform([](alliance_t detected) {
                                             return detected != auto_alliance;
                                         })
                                         .value_or(false);

    // bool bottom_wrong_color_detected =
    //   bottom_detected_color
    //     .transform([](alliance_t detected) {
    //         return detected != auto_alliance;
    //     })
    //     .value_or(false);

    // we have the wrong color, prcoess based on current state
    if (middle_wrong_color_detected || color_sort_one) {
        // try to outake through top of the intake since we are matchloading
        if (intake_state == intake && matchloader::get() == active &&
            is_driver == true) {
            // matchloading, should color sort through the back
            std::lock_guard lock(intake_mutex);

            // make sure no other balls are in the way since those would not
            // get color sorted out?
            bottom_motor.move(0);
            // move top most ball out through score side
            top_motor.move(127);
            setTopIntakePistonState(passthrough);

            // really long delay, could be really inconsistent
            pros::delay(800);
        } else if (intake_state == intake || intake_state == scoring_long ||
                   color_sort_one) {
            // need to take mutex
            std::lock_guard lock(intake_mutex);

            // move balls towards center hole
            setMiddleIntakePistonState(passthrough);
            // reverse ball a bit if touching top motor
            // bottom_motor.move(-100);
            // top_motor.move(-127);
            // pros::delay(120);
            // top_motor.move(-100);
            // pros::delay(200);
            // move bottom motor fast and no top motor
            bottom_motor.move(100);
            top_motor.move(50);
            pros::delay(90);
            top_motor.move(0);

            // delay some time to throw out ball
            pros::delay(100);
            // give slight time to take back ball that could have been taken
            // out
            setMiddleIntakePistonState(blocking);
            bottom_motor.move(-100);
            top_motor.move(0);
            pros::delay(100);

            if (color_sort_one) color_sort_one = false;
        }
        // else if (intake_state == scoring_middle ||
        //                     intake_state == slow_scoring_middle) {
        //              // need to take mutex
        //              std::lock_guard lock(intake_mutex);
        //
        // 	// assuming the balls come from the bottom,
        //
        //              // move balls up
        //              bottom_motor.move(-60);
        //              // move top most ball out
        //              top_motor.move(127);
        //
        // 	// wait for ball to go outwards
        //              pros::delay(200);
        //          }
    }
}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
    // intake update
    auto current_intake_state = intake_state;

    // only update motors if they are not being used elsewhere - waits for 2
    // millisecends to be able to use
    if (intake_mutex.take(2)) {
        setTopIntakePistonState(
          (intake_state == scoring_long ||
           intake_state == scoring_long_top_balls ||
           intake_state == scoring_long_top_balls_outake_bottom) ?
            passthrough :
            blocking);
        setMiddleIntakePistonState(
          (intake_state == intake_disabled_open_middle ||
           intake_state == scoring_middle_bottom_balls ||
           intake_state == scoring_middle_top_balls ||
           intake_state == scoring_middle_first_ball ||
           intake_state == scoring_middle_bottom_balls_slow ||
           intake_state == scoring_middle_top_balls_skills ||
           intake_state == scoring_middle_top_balls_skills_fast ||
           intake_state == scoring_middle_top_balls_skills_slow ||
           intake_state == outtake_bottom_balls_open_middle ||
           intake_state == scoring_middle ||
           intake_state == outtake_open_middle) ?
            passthrough :
            blocking);

        if (middle_active) {
            // if within first 400 msec then we are scoring top, otherwise
            // bottom

            Time timeout_time = skills_middle_scoring ? 2_sec : 2_sec;

            bool first_timeout = timeoutDone(timeout_time, *middle_active);

            if (skills_middle_scoring && !is_driver) {
                current_intake_state = first_timeout ?
                                         scoring_middle_top_balls :
                                         scoring_middle_bottom_balls;
            } else {
                current_intake_state = first_timeout ?
                                         scoring_middle_bottom_balls :
                                         scoring_middle_top_balls;
            }
        }

        if (current_intake_state == scoring_middle_bottom_balls) {
            if (timeoutDone(200_msec, last_ball_time)) {
                bottom_speed = bottom_motor_speeds[current_intake_state];
                top_speed = top_motor_speeds[current_intake_state];
            } else {
                bottom_speed = 127;
                top_speed = top_motor_speeds[current_intake_state];
            }
        } else {
            bottom_speed = bottom_motor_speeds[current_intake_state];
            top_speed = top_motor_speeds[current_intake_state];
        }

        bottom_motor.move(bottom_speed);
        top_motor.move(top_speed);

        // release mutex
        intake_mutex.give();
    }
}

// updates the state of the subsystem
void update() {
    // any code that needs to run regardless of driver mode can also run
    // here

    // update current detected color
    last_middle_detected_color = middle_detected_color;

    middle_detected_color = colorDetected(middle_intake_color_sensor);
    bottom_detected_color = colorDetected(bottom_intake_color_sensor);

    if (middle_detected_color) {
        last_ball_time = now();
    }

    if (is_driver) {
        driverUpdate();
    } else {
        autoUpdate();
    }
    hardwareUpdate();
}

void init(bool gdriver) {
    is_driver = gdriver;

    // set some hardware related options
    auto setup_color_sensor = [](pros::Optical& sensor) {
        if (sensor.is_installed()) {
            sensor.disable_gesture();
            sensor.set_integration_time(10);
            sensor.set_led_pwm(100);
        }
    };

    setup_color_sensor(middle_intake_color_sensor);
    setup_color_sensor(bottom_intake_color_sensor);

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once

    // pros::Task antijam_task(
    //   [] {
    //       while (true) {
    //           antiJam();
    //           pros::delay(10);
    //       }
    //   },
    //   "antijam");

    pros::Task colorsort_task(
      [] {
          while (true) {
              colorSort();
              pros::delay(10);
          }
      },
      "colorsort");

    pros::Task main_intake_task(
      [] {
          while (true) {
              update();
              pros::delay(10);
          }
      },
      "intake task");

    tasks_active = true;
}

// helper tasks to do common stuff
void in() {
    set(intake);
}

void out() {
    set(outtake);
}

void score_long() {
    set(scoring_long);
}

void score_middle() {
    set(scoring_middle);
}

void score_bottom() {
    set(scoring_bottom);
}
}; // namespace intake
