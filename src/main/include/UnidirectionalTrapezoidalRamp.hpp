#pragma once

#include <algorithm>
#include <chrono>

/** \file UnidirectionalTrapezoidalRamp.h
 *  \brief A header only implementation of a unidirectional trapezoidal ramp
 * 
 * This file contains an implementation of a ramp which only supports ramp-up (ramp-out isn't needed) and a statemachine style object which encapsulates the ramp for easier use
 */


/**
 * @brief Simple constexpr absolute value function for doubles
 * @param a 
 * @return absolute value of a
*/
constexpr double constexpr_abs(double a) {
	if (a < 0) return -a;
	return a;
};

/**
 * @brief Unidirectional Trapezoidal Ramp Class
 * 
 * The lower end is always 0
*/
class UnidirectionalTrapezoidalRamp {
private:

	double m_Endpoint, m_RampTime, m_RampActivationThreshold, m_RampSlope;

public:

	/**
	 * @param endpoint The upper endpoint of the ramp
	 * @param rampTime How long the ramp should go from 0 to the endpoint
	 * @param rampActivationThreshold How close should the feedback value be from the endpoint before just returning the endpoint (good for semiunstable systems).  Useful if you are using this for calculating a target and using an actual value as feedback.
	*/
	inline UnidirectionalTrapezoidalRamp(double endpoint, double rampTime, double rampActivationThreshold = 0.0f) : m_Endpoint{ endpoint }, m_RampTime{ rampTime }, m_RampActivationThreshold{ rampActivationThreshold }, m_RampSlope(endpoint / rampTime) { };

	/**
	 * @brief calculate the time into the ramp based upon the current value, maxing out at the ramp time
	*/
	[[nodiscard]] inline double calculateTimeFromCurrentValue(double currentValue) const noexcept {
		if (currentValue > m_Endpoint) return m_RampTime;
		return currentValue / m_RampSlope;
	};

	/**
	 * @brief naively calculate the current value from the current time (no feedback). if the time is past the ramp time, then this would return the endpoint.
	*/
	[[nodiscard]] inline double calculateValueFromTime(double currentTime) const noexcept {
		if (currentTime < m_RampTime) {
			return currentTime * m_RampSlope;
		}
		return m_Endpoint;
	};

	/**
	 * @brief Decide whether the ramp should be active based upon the feedback input.
	*/
	[[nodiscard]] inline bool shouldRamp(double actualCurrentValue) const noexcept {
		return constexpr_abs(m_Endpoint - actualCurrentValue) > m_RampActivationThreshold;
	};

	/**
	 * @brief calculate the current value based on the current time and the feedback from the system (used for the threshold, doesn't effect the actual ramping rate)
	*/
	[[nodiscard]] inline double calculateValueWithFeedback(double currentTime, double actualCurrentValue) const noexcept {
		if (shouldRamp(actualCurrentValue)) {
			return calculateValueFromTime(currentTime);
		}
		return m_Endpoint;
	};

	inline void setEndpoint(float end) { m_Endpoint = end; };
};

/**
 * @brief A class to encapsulate the ramp in an easy to use controller.
*/
class UnidirectionalTrapezoidalRampController {
private:
	using clock = std::chrono::high_resolution_clock;
	using duration = std::chrono::duration<double>;
	using time_point = std::chrono::time_point<clock, duration>;

	UnidirectionalTrapezoidalRamp m_Ramp;
	bool m_IsActive = false;
	time_point m_RampStartTime;

	static constexpr double kInactiveValue = 0.0f;

public:

	/**
	 * @see UnidirectionalTrapezoidalRamp
	*/
	inline UnidirectionalTrapezoidalRampController(double endpoint, double rampTime, double rampActivationThreshold = 0.0f) : m_Ramp(endpoint, rampTime, rampActivationThreshold) { };

	/**
	 * @brief start the ramp
	 * @param currentValue feedback used to calculate the real ramp start time
	*/
	inline void enter(double currentValue) {
		if (!m_IsActive) {
			m_IsActive = true;
		}

		if (currentValue == 0) {
			m_RampStartTime = clock::now();
		}
		else {
			double timeIntoRamp = m_Ramp.calculateTimeFromCurrentValue(currentValue);
			m_RampStartTime = clock::now() - duration(timeIntoRamp);
		}
	};

	/**
	 * @brief stop the ramp
	*/
	inline void exit() {
		m_IsActive = false;
	};

	/**
	 * @brief calculate the value given feedback
	 * @param currentValue feedback
	 * @return value
	*/
	inline double calculateValue(double currentValue) {
		if (m_IsActive) {
			return m_Ramp.calculateValueWithFeedback(std::chrono::duration_cast<duration>(clock::now() - m_RampStartTime).count(), currentValue);
		}

		return kInactiveValue;
	};

	/**
	 * @brief calculate the value naively (no feedback)
	 * @return value
	*/
	inline double calculateValue() {
		if (m_IsActive) {
			return m_Ramp.calculateValueFromTime((clock::now() - m_RampStartTime).count());
		}

		return kInactiveValue;
	};

	inline void setEndpoint(float end) { 
		m_Ramp.setEndpoint(end); 
	};

};