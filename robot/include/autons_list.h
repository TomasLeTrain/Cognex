#pragma once

#include "apis.h"
//

// includes only the auton list so that it can be included in files that never
// change (screen code)

#include <functional>
#include <string>

// maps the auton name to the corresponding function
extern std::unordered_map<std::string, std::function<void()>> auton_list;
