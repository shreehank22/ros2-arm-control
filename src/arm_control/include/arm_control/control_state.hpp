#pragma once

// Controller state machine: states, their names, and the transition rule.
// Kept free of ROS so it can be unit-tested directly.

enum class State {WAITING, FALLBACK, TRACKING};

// human-readable state name for logging
inline const char *stateName(State state) {
    switch (state) {
        case State::WAITING:  return "WAITING";
        case State::FALLBACK: return "FALLBACK";
        case State::TRACKING: return "TRACKING";
    }
    return "UNKNOWN";
}

// decides the next state from what has been received and how fresh it is
inline State nextState(bool has_measured, bool has_desired, bool measured_fresh, bool desired_fresh) {
    if (!has_measured) {
        return State::WAITING;
    }
    if (!measured_fresh || !desired_fresh || !has_desired) {
        return State::FALLBACK;
    }
    return State::TRACKING;
}
