#pragma once

#include "units/units.hpp"

namespace blazing {
namespace lyfast {

template<typename Input>
class mpFeedback {
  public:
    using VelT = Divided<Input, Time>;
    using AccelT = Divided<VelT, Time>;

  private:
    VelT m_max_vel;
    AccelT m_max_accel;

  public:
    mpFeedback(VelT max_vel, AccelT max_accel)
        : m_max_vel(max_vel),
          m_max_accel(max_accel) {}

    VelT update(Input measurement, Input target, Time dt) {
        const Input error = target - measurement;

        const Input decel_dist = units::square(m_max_vel) / (2 * m_max_accel);

        if (units::abs(error) > decel_dist) {
            return m_max_vel;
        } else {
            Exponentiated<VelT, std::ratio<2>> vi2 = units::square(m_max_vel);
            Exponentiated<VelT, std::ratio<2>> accel_term =
              (2 * m_max_accel * (decel_dist - error));
            Exponentiated<VelT, std::ratio<2>> diff = vi2 - accel_term;

            return units::sqrt(units::abs(diff)) * units::sgn(diff);
        }
    }

    void setMaxVel(VelT max_vel) {
        m_max_vel = max_vel;
    }

    void setMaxAccel(AccelT max_accel) {
        m_max_accel = max_accel;
    }

    VelT getMaxVel() {
        return m_max_vel;
    }

    AccelT getMaxAccel() {
        return m_max_accel;
    }
};
} // namespace lyfast
} // namespace blazing
