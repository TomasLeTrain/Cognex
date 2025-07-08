namespace intake {
enum intake_state_t { disabled = 0, forwards = 1, backwards = -1 };

/**
 * @brief set the direction of the intake, with an optional speed parameter to change the speed of the intake
 *
 * @param new_state new state of the intake
 * @param new_speed if specified, new speed of the intake
 */
void set(intake_state_t new_state, int new_speed = 127);

void init(bool gdriver);
} // namespace intake
