#pragma once

#include "lyfast/controllers/vel_controller.hpp"
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
    Time m_input_delay = 0_msec;

    Input m_low_threshold_error;
    Divided<VelT, Input> m_low_threshold_kp;

  public:
    mpFeedback(VelT max_vel,
               AccelT max_accel,
               Input low_threshold_error,
               Divided<VelT, Input> low_threshold_kp,
               Time input_delay)
        : m_max_vel(max_vel),
          m_max_accel(max_accel),
          m_low_threshold_error(low_threshold_error),
          m_low_threshold_kp(low_threshold_kp),
          m_input_delay(input_delay) {}

    VelT update(Input measurement, Input target, Time dt) {
        // signed error
        const Input error = target - measurement;
        const Number error_sgn = units::sgn(error);
        const Input abs_error = units::abs(error);

        const Exponentiated<VelT, std::ratio<2>> vi2 = units::square(m_max_vel);
        const AccelT a2 = 2 * m_max_accel;

        const Input decel_dist = vi2 / a2;

        const Input dx = decel_dist - error;
        const Exponentiated<VelT, std::ratio<2>> diff = vi2 - a2 * dx;
        const VelT abs_target_vel = units::sqrt(units::abs(diff));

        const Time t = (m_max_vel - abs_target_vel) / m_max_accel;

        // time at which error = 0
        const Time max_t = m_max_vel / m_max_accel;

        if (abs_error < m_low_threshold_error) {
            // error is already signed
            return m_low_threshold_kp * error;
        } else if (abs_error >= decel_dist) {
            return m_max_vel * error_sgn;
        } else {
            const Time projected_t = units::min(t + m_input_delay, max_t);
            // guaranteed to be positive since projected_t is capped
            const VelT projected_vel = m_max_vel - m_max_accel * projected_t;

            return projected_vel * error_sgn;
        }
    }

    void setMaxVel(VelT max_vel) {
        m_max_vel = max_vel;
    }

    void setMaxAccel(AccelT max_accel) {
        m_max_accel = max_accel;
    }

    void setLowThresholdError(Input threshold) {
        m_low_threshold_error = threshold;
    }

    void setLowThresholdKp(Divided<VelT, Input> kp) {
        m_low_threshold_kp = kp;
    }

    void setInputDelay(Time input_delay) {
        m_input_delay = input_delay;
    }

    VelT getMaxVel() {
        return m_max_vel;
    }

    AccelT getMaxAccel() {
        return m_max_accel;
    }

    Input getLowThresholdError() {
        return m_low_threshold_error;
    }

    Divided<VelT, Input> getLowThresholdKp() {
        return m_low_threshold_kp;
    }

    Time getInputDelay() {
        return m_input_delay;
    }
};
} // namespace lyfast
} // namespace blazing
