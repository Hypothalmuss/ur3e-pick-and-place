# Recording status published at a configurable rate for UI consumption.
# Use RELIABLE QoS on the publisher side so UI subscribers never miss updates.

builtin_interfaces/Time stamp

# Current recording state
bool is_recording
string task_name
string operator_name

# Cumulative counters
uint32 episode_count
uint32 current_timestep

# Sensor health indicators (watchdog-driven observability)
bool camera_alive
bool joint_states_alive

# Data quality metrics
float64 sync_offset_ms           # Latest sync offset between camera and joints (ms)
float64 avg_capture_rate_hz      # Measured capture rate over recent window (Hz)
uint32 dropped_frames            # Frames dropped in current episode due to sync/timeout
