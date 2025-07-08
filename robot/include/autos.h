// defines all the autons
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

// good includes for autons
#include "api.h"


#define NEW_AUTON(auton) namespace auton{ void run(); }
#define NEW_AUTONS(args...) NEW_AUTON(args)()
#define AUTON(auton) {#auton, auton::run },

// make sure autons are defined in both sections!
NEW_AUTON(auton1)

// std::map<std::string, std::function<void()>> auton_list = {
//
//     AUTON(auton1)
//     // AUTON(auton2)
//
// };
