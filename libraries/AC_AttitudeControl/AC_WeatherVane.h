#include <AP_Param/AP_Param.h>
#include <AP_InertialNav/AP_InertialNav.h>

// weather vane class
class AC_WeatherVane {
    public:

        // Constructor
        AC_WeatherVane(const AP_InertialNav& inav);

        // Do not allow copies
        AC_WeatherVane(const AC_WeatherVane &other) = delete;
        AC_WeatherVane &operator=(const AC_WeatherVane&) = delete;

        bool get_yaw_rate_cds(float& yaw_rate, const int16_t pilot_yaw, const int16_t roll_cdeg, const int16_t pitch_cdeg);

        // Use to relax weather vaning on landing.  Must be persistantly called before calls to get_weathervane_yaw_rate_cds().
        void set_relax(bool relax) { should_relax = relax; }

        void set_max_yaw_rate(float rate_max) { yaw_rate_max_deg_s = rate_max; }

        static AC_WeatherVane* get_singleton();

        static const struct AP_Param::GroupInfo var_info[];

    private:

        // References to other libraries
        const AP_InertialNav& _inav;

        // Different options for the direction that vehicle will turn into wind
        enum class Direction {
            OFF = 0,
            NOSE_IN = 1,
            NOSE_OR_TAIL_IN = 2,
            //TODO: Knife Edge for copter tailsitters
        };

        // Returns true if the vehicle is in a condition whereby weathervaning is allowed
        bool should_weathervane(int16_t pilot_yaw, int16_t roll_cdeg, int16_t pitch_cdeg, const uint32_t now);

        // Returns the set direction, handling the variable cast to type Direction
        Direction get_direction(void) const;

        // Function to reset all flags and set values. Involked whenever the weather vaning process is interupted
        void reset(void);

        /*
          Check if vehicle is above the minimum altitude to weather vane.
          Returns true if above the min set altitude.
          Uses terrain altitude if it is enabled.
        */
        bool below_min_alt(void);

        // Paramaters
        AP_Int8 _direction;
        AP_Float _gain;
        AP_Float _min_dz_ang_deg;
        AP_Int32 _min_alt;

        uint32_t last_pilot_input_ms;
        float last_output;
        bool should_relax;
        float yaw_rate_max_deg_s;
        bool active_msg_sent;
        uint32_t last_callback;
        uint32_t first_activate_ms;

        static AC_WeatherVane *_singleton;
};

namespace AP {
    AC_WeatherVane *weathervane();
};
