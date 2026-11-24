################################################################################
# micro T-Kernel 3.0 BSP makefile
#     Target Board: Raspberry Pi Pico
################################################################################

################################################################################
# Execution model
#
#   SMP=1  (default)  dual core.  TK_MAX_CORE is 2.
#   SMP=0             single core.  TK_MAX_CORE is 1, the per-core arrays
#                     collapse and the current-core index folds to a constant
#                     zero, so the generated code is equivalent to the
#                     pre-SMP port.
#
# Passed to the assembler as well: the dispatcher indexes the per-core arrays
# by SIO CPUID and needs the same profile.
################################################################################

SMP ?= 1

ifneq ($(SMP),0)
ifneq ($(SMP),1)
$(error Unknown SMP '$(SMP)'; use 0 or 1)
endif
endif

# Console default is needed here because the image name includes it.
#
# Defaults to usb_cdc so the console appears on the Pico's own USB cable.
# usb_cdc still mirrors every line to UART0 (GP0 TX / GP1 RX) for early boot
# and panic messages, so GP0/GP1 stay reserved either way.  CONSOLE=uart drops
# the USB console and its TinyUSB dependency.
CONSOLE ?= usb_cdc

# Pico W radio.  Defaults to cyw43: the radio is powered up and owned by a
# core-0 polling service task, using the Pico SDK's cyw43 driver.  WIFI=none
# leaves the radio powered down.
WIFI ?= cyw43
ifneq ($(filter $(WIFI),none cyw43),$(WIFI))
$(error Unknown WIFI '$(WIFI)'; use none or cyw43)
endif

ifeq ($(WIFI),cyw43)
WIFI_SUFFIX := _wifi
# SMP=0 is supported: the radio service task simply drops its TP_PRC1 affinity
# request when TK_SUPPORT_SMP is off (see cyw43_utk_kernel.c), the same way the
# USB console task already did. Everything touching the radio and lwIP still
# runs on that one task, which is what docs/RELEASE.md safety rule 6 actually
# requires -- single-core satisfies it trivially.
endif

# Optional lwIP network stack on top of WIFI=cyw43. NO_SYS=1 (raw API, no
# tcpip thread): everything that calls into lwIP -- cyw43_poll(),
# sys_check_timeouts(), and any application tcp_*/udp_*/dns_* call -- runs
# on the same processor-1-owned task that already services the radio, per
# docs/RELEASE.md safety rule 6. See lib/liblwip/sysdepend/pico_rp2040/.
# SENSOR_TEST=wifi tests the whole network path, so it turns lwIP and MQTT
# on unless the command line says otherwise.
ifeq ($(SENSOR_TEST),wifi)
NET ?= lwip
MQTT ?= 1
endif
NET ?= none
ifneq ($(filter $(NET),none lwip),$(NET))
$(error Unknown NET '$(NET)'; use none or lwip)
endif
ifeq ($(NET),lwip)
ifneq ($(WIFI),cyw43)
$(error NET=lwip requires WIFI=cyw43)
endif
NET_SUFFIX := _lwip
endif

# Optional MQTT client on top of NET=lwip. Connects and publishes from the
# same processor-1-owned task as everything else above -- see
# lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c. Broker address and topic
# come from config/wifi_secrets.h (gitignored; copy from
# config/wifi_secrets.example.h), same as the WiFi credentials.
MQTT ?= 0
ifneq ($(filter $(MQTT),0 1),$(MQTT))
$(error Unknown MQTT '$(MQTT)'; use 0 or 1)
endif
ifeq ($(MQTT),1)
ifneq ($(NET),lwip)
$(error MQTT=1 requires NET=lwip)
endif
MQTT_SUFFIX := _mqtt
endif

# TEST=1 builds the subsystem self-tests into the image (APP_TEST=1 in C).
# The image gets a _test suffix so it cannot be mistaken for the normal one.
TEST ?= 0
ifneq ($(filter $(TEST),0 1),$(TEST))
$(error Unknown TEST '$(TEST)'; use 0 or 1)
endif
ifeq ($(TEST),1)
TEST_SUFFIX := _test
endif

ifneq ($(filter-out 0,$(EXAMPLES)),)
$(error EXAMPLES is no longer supported; the template subsystem examples were removed)
endif

# Live encoder bring-up is separate from the unfinished vehicle application.
ODOMETRY_TEST ?= 0
ifneq ($(filter $(ODOMETRY_TEST),0 1),$(ODOMETRY_TEST))
$(error Unknown ODOMETRY_TEST '$(ODOMETRY_TEST)'; use 0 or 1)
endif
ifeq ($(ODOMETRY_TEST),1)
ifneq ($(TEST),1)
$(error ODOMETRY_TEST=1 requires TEST=1)
endif
ODOMETRY_SUFFIX := _odometry
endif

# VEHICLE=1 starts the vehicle application tasks at boot (APP_VEHICLE=1 in C).
# Startup still refuses to run until the hardware verification facts in
# app_program/app_main.c are confirmed on the robot.
VEHICLE ?= 0
ifneq ($(filter $(VEHICLE),0 1),$(VEHICLE))
$(error Unknown VEHICLE '$(VEHICLE)'; use 0 or 1)
endif
ifeq ($(VEHICLE),1)
ifneq ($(TEST),0)
$(error VEHICLE=1 cannot run with TEST=1: boot-time tests exercise the same task state)
endif
VEHICLE_SUFFIX := _vehicle
endif

# SENSOR_TEST selects one live hardware diagnostic; vehicle and test tasks stay off.
SENSOR_TEST ?= none
SENSOR_TEST_MODES := none ir imu imu_tilt imu_hump ultrasonic obstacle_width spin_heading motor_moves pid_tune spin_width diag_width line_follow junction wifi servo encoder line_barcode line barcode
ifneq ($(words $(SENSOR_TEST)),1)
$(error SENSOR_TEST must be exactly one of: $(SENSOR_TEST_MODES))
endif
ifeq ($(filter $(SENSOR_TEST_MODES),$(SENSOR_TEST)),)
$(error Unknown SENSOR_TEST '$(SENSOR_TEST)'; use one of: $(SENSOR_TEST_MODES))
endif
SERVO_TEST_US ?= 0
BARCODE_TEST_MODULE_COUNTS ?= 0
BARCODE_TEST_BLACK_ADC ?= 1000
BARCODE_TEST_WHITE_ADC ?= 600
BARCODE_TEST_DEBOUNCE_COUNTS ?= 4
# obstacle_width: confirm distance, and optional known object width for calibration.
OBSTACLE_DETECT_MM ?= 400
OBSTACLE_TRUE_WIDTH_MM ?= 0
# spin_heading (DRIVES MOTORS): wheel track and turning PWM percentages.
# Calibrated on the mat 2026-11-23: effective track 116 mm (ruler 110), heading
# within ~2 deg per full turn; 25/20 % keeps the stop overshoot to ~3.5 deg.
SPIN_TRACK_MM ?= 116
SPIN_PWM ?= 25
SPIN_PWM_SLOW ?= 20
SPIN_RIGHT_TRIM ?= 0
# motor_moves (DRIVES MOTORS): open-loop PWM %, per-wheel trim %, straight
# distance, wheel track and inner-wheel share % for the arcs.
MOVE_PWM ?= 50
MOVE_TRIM_L ?= 100
MOVE_TRIM_R ?= 100
MOVE_DIST_MM ?= 500
MOVE_TRACK_MM ?= 110
MOVE_ARC_INNER ?= 50
MOVE_SKIP_BALANCE ?= 0
# pid_tune (DRIVES MOTORS): wheel speed PID gains (decimals allowed), target
# cm/s, base and max PWM %, update period, floor distance, in-air run time,
# per-sample trace (0/1) and arc steering cm/s (0 skips the arc steps).
PID_KP ?= 2.0
PID_KI ?= 0.0
PID_KD ?= 0.0
PID_SPEED ?= 15
PID_BASE ?= 20
PID_MAX ?= 60
PID_PERIOD_MS ?= 50
PID_DIST_MM ?= 400
PID_AIR_MS ?= 3000
PID_TRACE ?= 1
PID_TURN ?= 4
# pid_tune wheel feed-forward %PWM per cm/s around PID_BASE at PID_SPEED (0 =
# flat base; PID_FF=1.4 PID_BASE=22 PID_SPEED=15 matches the vehicle). The arc
# steps print the slope the wheels actually needed.
PID_FF ?= 0
# spin_width (DRIVES MOTORS, uses the spin_heading SPIN_* values): turn limit
# each way, measuring step and edge-finding step in degrees, sensor distance
# ahead of the wheel axle, obstacle cluster tolerance, free gap needed beside
# the obstacle (robot width + clearance), how far beyond the obstacle side
# objects still count (assumed depth + robot length + margin), beam trim in degrees (decimals
# allowed), an optional known object width for calibration, and SCAN_FULL=1 to
# always sweep the whole R..L SCAN_HALF_DEG instead of finding the edges first.
# SCAN_BYPASS=1 then pulls out to the chosen side (right unless the obstacle
# sticks out further right): car width, side gap to keep, extra margin for the
# scan's width error, and the sideways speed. SCAN_REJOIN=1 then drives past it,
# looking across at the line from SCAN_MIN_DEPTH_MM past the face and every
# SCAN_PASS_STEP_MM (giving up after SCAN_MAX_DEPTH_MM), and rejoins the line.
SCAN_HALF_DEG ?= 60
SCAN_STEP_DEG ?= 3
SCAN_FIND_STEP_DEG ?= 6
SCAN_LANE_MM ?= 200
SCAN_BEYOND_MM ?= 400
SCAN_MAX_OBSTACLE_MM ?= 450
SCAN_FULL ?= 0
SCAN_BYPASS ?= 1
SCAN_CAR_WIDTH_MM ?= 140
SCAN_CLEAR_MM ?= 50
SCAN_MARGIN_MM ?= 80
SCAN_SPEED_CM_S ?= 15
SCAN_REJOIN ?= 1
SCAN_MIN_DEPTH_MM ?= 50
SCAN_PASS_STEP_MM ?= 100
SCAN_MAX_DEPTH_MM ?= 600
# diag_width (DRIVES MOTORS, uses the spin_heading SPIN_* values): pass angle
# (heading left, servo right; servo limit 20), speed along the path, straight
# back-up before the pass, car width, side gap to keep, pull out after
# measuring (0/1), and an optional known width.
DIAG_ANGLE_DEG ?= 20
DIAG_SPEED_CM_S ?= 15
DIAG_BACKUP_MM ?= 100
DIAG_CAR_WIDTH_MM ?= 140
DIAG_CLEAR_MM ?= 50
DIAG_PULLOUT ?= 1
DIAG_TRUE_WIDTH_MM ?= 0
# line_follow (DRIVES MOTORS): line steering speed, full turn, first-correction
# turn and the ms it takes to build from one to the other (override
# line_steering.h for this build; LF_SPEED + LF_TURN <= 20; LF_TURN_MIN=LF_TURN
# is the old fixed bang-bang), wheel feed-forward %PWM per cm/s (0 = old flat
# base), run length, steering period (= controller), trace (0/1), tightest
# curve radius on the mat (0 = unknown; enables the "turn needed" check) and
# effective wheel track.
LF_SPEED ?= 12
LF_TURN ?= 7
LF_TURN_MIN ?= 2
LF_RAMP_MS ?= 400
LF_FF ?= 1.4
# Ramp-mode (SLOPE) profile, used by the vehicle while the IMU reports a ramp;
# LF_SLOPE=1 runs it for the whole line_follow run (the test has no IMU).
LF_SLOPE ?= 0
LF_SLOPE_SPEED ?= 9
LF_SLOPE_TURN ?= 5
LF_SLOPE_TURN_MIN ?= 2
LF_SLOPE_RAMP_MS ?= 400
LF_DIST_MM ?= 1500
LF_STEER_MS ?= 50
LF_TRACE ?= 1
LF_CURVE_R_MM ?= 0
LF_TRACK_MM ?= 116
# LF_BARCODE=1 also decodes barcodes while following, with two decoders on the
# same pass: A as the vehicle (20 ms samples, black > 1000) and B fast (2 ms,
# hysteresis, debounce); LF_BC_MODULE = counts per narrow module (vehicle: 35).
LF_BARCODE ?= 0
LF_BC_MODULE ?= 35
# Decoder C measures distance at the barcode sensor, this far left of centre.
LF_BC_SENSOR_Y_MM ?= 40
# junction (DRIVES MOTORS): follows the line with the line_steering.h settings
# and detects a tape crossing the path with the left (barcode) sensor + the
# line sensors. JN_ACTION = stop | straight | left | right | uturn. Turns drive
# JN_FORWARD_MM first (~ line sensors to wheel axle), spin by the encoders
# (braking JN_TURN_TRIM_DEG early), then follow the new line JN_AFTER_MM.
# Detector: left black JN_LEFT_MIN_MM..JN_LEFT_MAX_MM, then centre+right both
# black >= JN_BOTH_MIN_MM from JN_OFFSET_MM after it (+/- JN_TOL_MM).
# Widths are as the sensors measure them: 18 mm tape reads ~12 mm, and the
# line sensors reach it ~8 mm after the left one (measured).
JN_ACTION ?= stop
JN_FORWARD_MM ?= 100
# Right turns: the tracked tape edge is off the robot centreline, so they need
# a different forward distance (2x that offset apart from left turns).
JN_FORWARD_RIGHT_MM ?= 120
# JN_SEEK=1 turns until a line sensor finds the new line (looking from
# JN_SEEK_BEFORE_DEG before to JN_SEEK_AFTER_DEG past the target angle);
# JN_SEEK=0 spins a fixed angle, braking JN_TURN_TRIM_DEG early.
JN_SEEK ?= 1
JN_SEEK_BEFORE_DEG ?= 30
JN_SEEK_AFTER_DEG ?= 30
JN_AFTER_MM ?= 400
JN_RUN_MM ?= 1500
JN_TURN_TRIM_DEG ?= 4
JN_TRACE ?= 1
JN_LEFT_MIN_MM ?= 8
JN_LEFT_MAX_MM ?= 80
JN_OFFSET_MM ?= 8
JN_TOL_MM ?= 12
JN_BOTH_MIN_MM ?= 5
# No-line start (line sensors white at GO, e.g. only a crossing tape on the
# mat): drive straight up to JN_BLIND_MM to find it, backing off it first if
# the left sensor starts on it.
JN_BLIND_MM ?= 200
SCAN_SENSOR_OFFSET_MM ?= 70
SCAN_CLUSTER_MM ?= 60
SCAN_TRIM_DEG ?= 11.6
SCAN_TRUE_WIDTH_MM ?= 0
ifneq ($(SENSOR_TEST),none)
ifneq ($(TEST)$(ODOMETRY_TEST)$(VEHICLE),000)
$(error SENSOR_TEST cannot be combined with TEST, ODOMETRY_TEST, or VEHICLE)
endif
SENSOR_TEST_SUFFIX := _sensor_$(SENSOR_TEST)
endif
ifeq ($(SENSOR_TEST),ir)
SENSOR_TEST_VALUE := 1
else ifeq ($(SENSOR_TEST),imu)
SENSOR_TEST_VALUE := 2
else ifeq ($(SENSOR_TEST),ultrasonic)
SENSOR_TEST_VALUE := 3
else ifeq ($(SENSOR_TEST),obstacle_width)
SENSOR_TEST_VALUE := 11
else ifeq ($(SENSOR_TEST),spin_heading)
SENSOR_TEST_VALUE := 12
else ifeq ($(SENSOR_TEST),motor_moves)
SENSOR_TEST_VALUE := 13
else ifeq ($(SENSOR_TEST),pid_tune)
SENSOR_TEST_VALUE := 14
else ifeq ($(SENSOR_TEST),spin_width)
SENSOR_TEST_VALUE := 15
else ifeq ($(SENSOR_TEST),diag_width)
SENSOR_TEST_VALUE := 16
else ifeq ($(SENSOR_TEST),line_follow)
SENSOR_TEST_VALUE := 17
else ifeq ($(SENSOR_TEST),junction)
SENSOR_TEST_VALUE := 18
else ifeq ($(SENSOR_TEST),wifi)
SENSOR_TEST_VALUE := 19
ifneq ($(MQTT),1)
$(error SENSOR_TEST=wifi needs WIFI=cyw43 NET=lwip MQTT=1)
endif
else ifeq ($(SENSOR_TEST),servo)
SENSOR_TEST_VALUE := 4
else ifeq ($(SENSOR_TEST),encoder)
SENSOR_TEST_VALUE := 5
else ifeq ($(SENSOR_TEST),imu_tilt)
SENSOR_TEST_VALUE := 6
else ifeq ($(SENSOR_TEST),imu_hump)
SENSOR_TEST_VALUE := 10
else ifeq ($(SENSOR_TEST),line_barcode)
SENSOR_TEST_VALUE := 7
else ifeq ($(SENSOR_TEST),line)
SENSOR_TEST_VALUE := 8
else ifeq ($(SENSOR_TEST),barcode)
SENSOR_TEST_VALUE := 9
else
SENSOR_TEST_VALUE := 0
endif
# CRASH_REPORT=1 also copies every console line to UART0 on GP0 (TX) /
# GP1 (RX), 115200 8N1.  UART output is sent immediately, so it still shows
# the last messages and the kernel's crash report after a crash, when the
# USB console can no longer send anything.  It needs a USB-serial adapter on
# GP0/GP1 and takes those two pins.
# CRASH_REPORT=0 (default): console on USB only; the boot code leaves GP0/GP1
# alone, so a subsystem may claim them in TEAM.md.
# CONSOLE=uart always uses UART0 as the console, so it forces CRASH_REPORT=1.
CRASH_REPORT ?= 0
ifneq ($(filter $(CRASH_REPORT),0 1),$(CRASH_REPORT))
$(error Unknown CRASH_REPORT '$(CRASH_REPORT)'; use 0 or 1)
endif
ifeq ($(CONSOLE),uart)
override CRASH_REPORT := 1
endif
ifeq ($(CONSOLE)$(CRASH_REPORT),usb_cdc1)
CRASH_SUFFIX := _crash
endif

#
# Images are named after the profile they were built with.  Every image is
# flashed by hand from a downloads folder, so identically-named artifacts are
# a live hazard: an SMP=1 build was once flashed in place of an SMP=0 one and
# only the banner in the log revealed it.
#
EXE_FILE := mtk3pico_smp$(SMP)_$(CONSOLE)$(WIFI_SUFFIX)$(NET_SUFFIX)$(MQTT_SUFFIX)$(CRASH_SUFFIX)$(VEHICLE_SUFFIX)$(ODOMETRY_SUFFIX)$(TEST_SUFFIX)$(SENSOR_TEST_SUFFIX)

GCC := arm-none-eabi-gcc
AS := arm-none-eabi-gcc
LINK := arm-none-eabi-gcc
LDLIBS += -lm
SIZE := arm-none-eabi-size
E2U := tools/elf2uf2

# Fail early, with a readable message, when the cross compiler is not on PATH.
ifneq ($(MAKECMDGOALS),clean)
ifeq ($(shell command -v $(GCC) 2>/dev/null),)
$(error $(GCC) not found. Add your Arm GNU Toolchain 'bin' folder to PATH, e.g. export PATH="/path/to/arm-gnu-toolchain/bin:$$PATH")
endif
endif

# The USB console and the radio take sources from the Pico C SDK.  Its location
# must come from the environment (export PICO_SDK_PATH=...) or the command line.
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(CONSOLE)$(WIFI),uartnone)
ifeq ($(strip $(PICO_SDK_PATH)),)
$(error PICO_SDK_PATH is not set. export PICO_SDK_PATH=/path/to/pico-sdk, or build with CONSOLE=uart WIFI=none)
endif
ifeq ($(wildcard $(PICO_SDK_PATH)/pico_sdk_init.cmake),)
$(error PICO_SDK_PATH='$(PICO_SDK_PATH)' does not look like a Pico C SDK folder)
endif
endif
endif

# The ELF-to-UF2 converter is a small host program built from tools/elf2uf2
# with the host C++ compiler the first time it is needed.
$(E2U): ../tools/elf2uf2/main.cpp
	@echo 'Building host tool: $@'
	c++ -O2 -std=c++14 -I../tools/elf2uf2 -o $@ $<

$(EXE_FILE).elf: | $(E2U)

CFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -std=gnu11 \
    -O2 -g3 \
    -MMD -MP \
    -mfloat-abi=soft \

ASFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -x assembler-with-cpp \
    -O2 -g3 \
    -MMD -MP \

LFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding \
    -nostartfiles \
    -O2 -g3 \
    -mfloat-abi=soft \

LNKFILE := "../etc/linker/pico_rp2040/tkernel_map.ld"

# Applied here, after CFLAGS/ASFLAGS are assigned with ':=', which would
# otherwise discard anything appended above.
ifeq ($(SMP),1)
CFLAGS  += -DCNF_SMP=1
ASFLAGS += -DCNF_SMP=1
endif
CFLAGS  += -DAPP_TEST=$(TEST)
CFLAGS  += -DAPP_ODOMETRY_TEST=$(ODOMETRY_TEST)
CFLAGS  += -DAPP_VEHICLE=$(VEHICLE)
CFLAGS  += -DAPP_SENSOR_TEST=$(SENSOR_TEST_VALUE)
ifeq ($(SENSOR_TEST),barcode)
CFLAGS  += -DBARCODE_TEST_MODULE_COUNTS=$(BARCODE_TEST_MODULE_COUNTS)
CFLAGS  += -DBARCODE_TEST_BLACK_ADC=$(BARCODE_TEST_BLACK_ADC)
CFLAGS  += -DBARCODE_TEST_WHITE_ADC=$(BARCODE_TEST_WHITE_ADC)
CFLAGS  += -DBARCODE_TEST_DEBOUNCE_COUNTS=$(BARCODE_TEST_DEBOUNCE_COUNTS)
endif
ifeq ($(SENSOR_TEST),servo)
CFLAGS  += -DSERVO_TEST_US=$(SERVO_TEST_US)
endif
ifeq ($(SENSOR_TEST),obstacle_width)
CFLAGS  += -DOBS_WIDTH_TEST_DETECT_MM=$(OBSTACLE_DETECT_MM)U
CFLAGS  += -DOBS_WIDTH_TEST_TRUE_WIDTH_MM=$(OBSTACLE_TRUE_WIDTH_MM)U
endif
ifeq ($(SENSOR_TEST),spin_heading)
CFLAGS  += -DSPIN_TRACK_MM=$(SPIN_TRACK_MM)U -DSPIN_PWM=$(SPIN_PWM)U -DSPIN_PWM_SLOW=$(SPIN_PWM_SLOW)U
CFLAGS  += -DSPIN_RIGHT_TRIM=$(SPIN_RIGHT_TRIM)
endif
ifeq ($(SENSOR_TEST),motor_moves)
CFLAGS  += -DMOVE_PWM=$(MOVE_PWM)U -DMOVE_TRIM_L=$(MOVE_TRIM_L)U -DMOVE_TRIM_R=$(MOVE_TRIM_R)U
CFLAGS  += -DMOVE_DIST_MM=$(MOVE_DIST_MM)U -DMOVE_TRACK_MM=$(MOVE_TRACK_MM)U -DMOVE_ARC_INNER=$(MOVE_ARC_INNER)U -DMOVE_SKIP_BALANCE=$(MOVE_SKIP_BALANCE)U
endif
ifeq ($(SENSOR_TEST),pid_tune)
CFLAGS  += -DPID_KP=$(PID_KP) -DPID_KI=$(PID_KI) -DPID_KD=$(PID_KD)
CFLAGS  += -DPID_SPEED=$(PID_SPEED)U -DPID_BASE=$(PID_BASE)U -DPID_MAX=$(PID_MAX)U
CFLAGS  += -DPID_PERIOD_MS=$(PID_PERIOD_MS)U -DPID_DIST_MM=$(PID_DIST_MM)U
CFLAGS  += -DPID_AIR_MS=$(PID_AIR_MS)U -DPID_TRACE=$(PID_TRACE)U -DPID_TURN=$(PID_TURN)U
CFLAGS  += -DPID_FF=$(PID_FF)
endif
ifeq ($(SENSOR_TEST),spin_width)
CFLAGS  += -DSPIN_TRACK_MM=$(SPIN_TRACK_MM)U -DSPIN_PWM=$(SPIN_PWM)U -DSPIN_PWM_SLOW=$(SPIN_PWM_SLOW)U
CFLAGS  += -DSCAN_HALF_DEG=$(SCAN_HALF_DEG)U -DSCAN_STEP_DEG=$(SCAN_STEP_DEG)U
CFLAGS  += -DSCAN_SENSOR_OFFSET_MM=$(SCAN_SENSOR_OFFSET_MM)U -DSCAN_CLUSTER_MM=$(SCAN_CLUSTER_MM)U
CFLAGS  += -DSCAN_TRIM_DEG=$(SCAN_TRIM_DEG) -DSCAN_TRUE_WIDTH_MM=$(SCAN_TRUE_WIDTH_MM)U
CFLAGS  += -DSCAN_FIND_STEP_DEG=$(SCAN_FIND_STEP_DEG)U -DSCAN_LANE_MM=$(SCAN_LANE_MM)U -DSCAN_FULL=$(SCAN_FULL)
CFLAGS  += -DSCAN_BEYOND_MM=$(SCAN_BEYOND_MM)U -DSCAN_MAX_OBSTACLE_MM=$(SCAN_MAX_OBSTACLE_MM)U
CFLAGS  += -DSCAN_BYPASS=$(SCAN_BYPASS) -DSCAN_CAR_WIDTH_MM=$(SCAN_CAR_WIDTH_MM)U -DSCAN_CLEAR_MM=$(SCAN_CLEAR_MM)U
CFLAGS  += -DSCAN_MARGIN_MM=$(SCAN_MARGIN_MM)U -DSCAN_SPEED_CM_S=$(SCAN_SPEED_CM_S)U
CFLAGS  += -DSCAN_REJOIN=$(SCAN_REJOIN) -DSCAN_MIN_DEPTH_MM=$(SCAN_MIN_DEPTH_MM)U
CFLAGS  += -DSCAN_PASS_STEP_MM=$(SCAN_PASS_STEP_MM)U -DSCAN_MAX_DEPTH_MM=$(SCAN_MAX_DEPTH_MM)U
endif
ifeq ($(SENSOR_TEST),diag_width)
CFLAGS  += -DSPIN_TRACK_MM=$(SPIN_TRACK_MM)U -DSPIN_PWM=$(SPIN_PWM)U -DSPIN_PWM_SLOW=$(SPIN_PWM_SLOW)U
CFLAGS  += -DDIAG_ANGLE_DEG=$(DIAG_ANGLE_DEG)U -DDIAG_SPEED_CM_S=$(DIAG_SPEED_CM_S)U
CFLAGS  += -DDIAG_CAR_WIDTH_MM=$(DIAG_CAR_WIDTH_MM)U -DDIAG_CLEAR_MM=$(DIAG_CLEAR_MM)U
CFLAGS  += -DDIAG_PULLOUT=$(DIAG_PULLOUT) -DDIAG_BACKUP_MM=$(DIAG_BACKUP_MM)U
CFLAGS  += -DDIAG_TRUE_WIDTH_MM=$(DIAG_TRUE_WIDTH_MM)U
endif
ifeq ($(SENSOR_TEST),line_follow)
CFLAGS  += -DLINE_STEERING_SPEED_CM_S=$(LF_SPEED) -DLINE_STEERING_TURN_CM_S=$(LF_TURN)
CFLAGS  += -DLINE_STEERING_TURN_MIN_CM_S=$(LF_TURN_MIN) -DLINE_STEERING_RAMP_MS=$(LF_RAMP_MS)
CFLAGS  += -DLF_FF=$(LF_FF) -DLF_SLOPE=$(LF_SLOPE)
CFLAGS  += -DLF_BARCODE=$(LF_BARCODE) -DLF_BC_MODULE=$(LF_BC_MODULE)U -DLF_BC_SENSOR_Y_MM=$(LF_BC_SENSOR_Y_MM)
CFLAGS  += -DLINE_STEERING_SLOPE_SPEED_CM_S=$(LF_SLOPE_SPEED) -DLINE_STEERING_SLOPE_TURN_CM_S=$(LF_SLOPE_TURN)
CFLAGS  += -DLINE_STEERING_SLOPE_TURN_MIN_CM_S=$(LF_SLOPE_TURN_MIN) -DLINE_STEERING_SLOPE_RAMP_MS=$(LF_SLOPE_RAMP_MS)
CFLAGS  += -DLF_DIST_MM=$(LF_DIST_MM)U -DLF_STEER_MS=$(LF_STEER_MS)U -DLF_TRACE=$(LF_TRACE)U
CFLAGS  += -DLF_CURVE_R_MM=$(LF_CURVE_R_MM)U -DLF_TRACK_MM=$(LF_TRACK_MM)U
endif
ifeq ($(SENSOR_TEST),junction)
JN_ACTION_MODES := stop straight left right uturn
ifeq ($(filter $(JN_ACTION_MODES),$(JN_ACTION)),)
$(error Unknown JN_ACTION '$(JN_ACTION)'; use one of: $(JN_ACTION_MODES))
endif
JN_ACTION_VALUE := $(if $(filter stop,$(JN_ACTION)),0,$(if $(filter straight,$(JN_ACTION)),1,$(if $(filter left,$(JN_ACTION)),2,$(if $(filter right,$(JN_ACTION)),3,4))))
CFLAGS  += -DJN_ACTION=$(JN_ACTION_VALUE) -DJN_FORWARD_MM=$(JN_FORWARD_MM)U -DJN_FORWARD_RIGHT_MM=$(JN_FORWARD_RIGHT_MM)U -DJN_AFTER_MM=$(JN_AFTER_MM)U
CFLAGS  += -DJN_RUN_MM=$(JN_RUN_MM)U -DJN_TURN_TRIM_DEG=$(JN_TURN_TRIM_DEG) -DJN_TRACE=$(JN_TRACE)U
CFLAGS  += -DJN_LEFT_MIN_MM=$(JN_LEFT_MIN_MM) -DJN_LEFT_MAX_MM=$(JN_LEFT_MAX_MM)
CFLAGS  += -DJN_OFFSET_MM=$(JN_OFFSET_MM) -DJN_TOL_MM=$(JN_TOL_MM) -DJN_BOTH_MIN_MM=$(JN_BOTH_MIN_MM)
CFLAGS  += -DJN_BLIND_MM=$(JN_BLIND_MM)U -DJN_SEEK=$(JN_SEEK)
CFLAGS  += -DJN_SEEK_BEFORE_DEG=$(JN_SEEK_BEFORE_DEG) -DJN_SEEK_AFTER_DEG=$(JN_SEEK_AFTER_DEG)
endif
CFLAGS  += -DTM_CONSOLE_UART=$(CRASH_REPORT)
ifeq ($(CRASH_REPORT),0)
CFLAGS  += -DBOARD_UART0_FREE=1
endif

################################################################################
# Console selection
#
#   CONSOLE=usb_cdc             USB CDC-ACM console, with the UART0 mirror
#                               retained for early boot and panics.
#   CONSOLE=uart                UART0 only, no TinyUSB dependency.
#
# usb_cdc pulls seven TinyUSB sources and a set of pico-sdk headers from
# PICO_SDK_PATH.  Only headers and those sources are used; the port does not
# link the pico-sdk runtime, and lib/libtm/sysdepend/pico_rp2040/usb/
# usb_sdk_compat.c supplies the few SDK hooks TinyUSB calls, mapped onto
# micro T-Kernel's tk_def_int().
################################################################################

################################################################################
# Stale-object guard
#
# Every profile compiles into the same object tree, so switching profiles
# without cleaning silently relinks the previous profile's objects: SMP=0 and
# SMP=1 built back to back produced byte-identical images. Record the profile
# and discard the objects whenever it changes. Only *.o and *.d are removed:
# build_make/mtkernel_3/ also holds the tracked subdir.mk files, so removing
# the directory wholesale deletes source. Images are named after their
# profile, so both can coexist. This runs at parse time, so it completes
# before any recipe executes.
################################################################################

PROFILE_ID := smp$(SMP)-$(CONSOLE)-wifi$(WIFI)-net$(NET)-mqtt$(MQTT)-crash$(CRASH_REPORT)-vehicle$(VEHICLE)-odometry$(ODOMETRY_TEST)-test$(TEST)-sensor$(SENSOR_TEST)-servo-pulse$(SERVO_TEST_US)
ifeq ($(SENSOR_TEST),barcode)
PROFILE_ID := $(PROFILE_ID)-barcode-module$(BARCODE_TEST_MODULE_COUNTS)-adc$(BARCODE_TEST_WHITE_ADC)-$(BARCODE_TEST_BLACK_ADC)-debounce$(BARCODE_TEST_DEBOUNCE_COUNTS)
endif
ifeq ($(SENSOR_TEST),obstacle_width)
PROFILE_ID := $(PROFILE_ID)-detect$(OBSTACLE_DETECT_MM)-truewidth$(OBSTACLE_TRUE_WIDTH_MM)
endif
ifeq ($(SENSOR_TEST),spin_heading)
PROFILE_ID := $(PROFILE_ID)-track$(SPIN_TRACK_MM)-pwm$(SPIN_PWM)-$(SPIN_PWM_SLOW)-trim$(SPIN_RIGHT_TRIM)
endif
ifeq ($(SENSOR_TEST),motor_moves)
PROFILE_ID := $(PROFILE_ID)-pwm$(MOVE_PWM)-trim$(MOVE_TRIM_L)-$(MOVE_TRIM_R)-dist$(MOVE_DIST_MM)-track$(MOVE_TRACK_MM)-arc$(MOVE_ARC_INNER)-skipbal$(MOVE_SKIP_BALANCE)
endif
ifeq ($(SENSOR_TEST),pid_tune)
PROFILE_ID := $(PROFILE_ID)-kp$(PID_KP)-ki$(PID_KI)-kd$(PID_KD)-v$(PID_SPEED)-base$(PID_BASE)-max$(PID_MAX)-T$(PID_PERIOD_MS)-dist$(PID_DIST_MM)-air$(PID_AIR_MS)-trace$(PID_TRACE)-turn$(PID_TURN)-ff$(PID_FF)
endif
ifeq ($(SENSOR_TEST),spin_width)
PROFILE_ID := $(PROFILE_ID)-track$(SPIN_TRACK_MM)-pwm$(SPIN_PWM)-$(SPIN_PWM_SLOW)-half$(SCAN_HALF_DEG)-step$(SCAN_STEP_DEG)-off$(SCAN_SENSOR_OFFSET_MM)-cl$(SCAN_CLUSTER_MM)-trim$(SCAN_TRIM_DEG)-true$(SCAN_TRUE_WIDTH_MM)-find$(SCAN_FIND_STEP_DEG)-lane$(SCAN_LANE_MM)-beyond$(SCAN_BEYOND_MM)-maxobs$(SCAN_MAX_OBSTACLE_MM)-full$(SCAN_FULL)-byp$(SCAN_BYPASS)-car$(SCAN_CAR_WIDTH_MM)-clr$(SCAN_CLEAR_MM)-mrg$(SCAN_MARGIN_MM)-v$(SCAN_SPEED_CM_S)-rej$(SCAN_REJOIN)-md$(SCAN_MIN_DEPTH_MM)-ps$(SCAN_PASS_STEP_MM)-xd$(SCAN_MAX_DEPTH_MM)
endif
ifeq ($(SENSOR_TEST),diag_width)
PROFILE_ID := $(PROFILE_ID)-track$(SPIN_TRACK_MM)-pwm$(SPIN_PWM)-$(SPIN_PWM_SLOW)-ang$(DIAG_ANGLE_DEG)-v$(DIAG_SPEED_CM_S)-car$(DIAG_CAR_WIDTH_MM)-clr$(DIAG_CLEAR_MM)-out$(DIAG_PULLOUT)-bak$(DIAG_BACKUP_MM)-true$(DIAG_TRUE_WIDTH_MM)
endif
ifeq ($(SENSOR_TEST),line_follow)
PROFILE_ID := $(PROFILE_ID)-v$(LF_SPEED)-turn$(LF_TURN)-min$(LF_TURN_MIN)-ramp$(LF_RAMP_MS)-ff$(LF_FF)-slope$(LF_SLOPE)-s$(LF_SLOPE_SPEED)-$(LF_SLOPE_TURN)-$(LF_SLOPE_TURN_MIN)-$(LF_SLOPE_RAMP_MS)-dist$(LF_DIST_MM)-steer$(LF_STEER_MS)-trace$(LF_TRACE)-curve$(LF_CURVE_R_MM)-track$(LF_TRACK_MM)-bc$(LF_BARCODE)-$(LF_BC_MODULE)-y$(LF_BC_SENSOR_Y_MM)
endif
ifeq ($(SENSOR_TEST),junction)
PROFILE_ID := $(PROFILE_ID)-act$(JN_ACTION)-fwd$(JN_FORWARD_MM)-$(JN_FORWARD_RIGHT_MM)-after$(JN_AFTER_MM)-run$(JN_RUN_MM)-trim$(JN_TURN_TRIM_DEG)-trace$(JN_TRACE)-l$(JN_LEFT_MIN_MM)-$(JN_LEFT_MAX_MM)-o$(JN_OFFSET_MM)-t$(JN_TOL_MM)-b$(JN_BOTH_MIN_MM)-blind$(JN_BLIND_MM)-seek$(JN_SEEK)-$(JN_SEEK_BEFORE_DEG)-$(JN_SEEK_AFTER_DEG)
endif
PROFILE_FILE := .build_profile

ifneq ($(strip $(shell cat $(PROFILE_FILE) 2>/dev/null)),$(PROFILE_ID))
$(info Build profile is now $(PROFILE_ID); discarding objects built for another profile.)
$(shell find mtkernel_3 -name '*.o' -o -name '*.d' | xargs -r rm -f; \
	  rm -f $(EXE_FILE).elf $(EXE_FILE).map $(EXE_FILE).uf2; \
	  echo $(PROFILE_ID) > $(PROFILE_FILE))
endif


ifeq ($(CONSOLE),usb_cdc)

TINYUSB_PATH := $(PICO_SDK_PATH)/lib/tinyusb

ifeq ($(wildcard $(TINYUSB_PATH)/src/tusb.c),)
$(error TinyUSB not found at $(TINYUSB_PATH). Set PICO_SDK_PATH, or build with CONSOLE=uart)
endif

USBDIR := ../lib/libtm/sysdepend/pico_rp2040/usb

USB_SRCS := usb_console.c usb_descriptors.c usb_sdk_compat.c usb_tinyusb_glue.c
USB_OBJS := $(addprefix ./mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/,$(USB_SRCS:.c=.o))

TINYUSB_SRCS := tusb.c \
                common/tusb_fifo.c \
                device/usbd.c \
                device/usbd_control.c \
                class/cdc/cdc_device.c \
                portable/raspberrypi/rp2040/rp2040_usb.c \
                portable/raspberrypi/rp2040/dcd_rp2040.c
TINYUSB_OBJS := $(addprefix ./mtkernel_3/tinyusb/,$(TINYUSB_SRCS:.c=.o))

USB_INCLUDES := -I"$(USBDIR)" \
                -I"$(TINYUSB_PATH)/src" \
                -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include"

CFLAGS += -DTM_CONSOLE_USB_CDC=1 -DNDEBUG $(USB_INCLUDES)

OBJS += $(USB_OBJS) $(TINYUSB_OBJS)
C_DEPS += $(USB_OBJS:.o=.d) $(TINYUSB_OBJS:.o=.d)

mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/%.o: $(USBDIR)/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/tinyusb/%.o: $(TINYUSB_PATH)/src/%.c
	@echo 'Building TinyUSB file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

else ifneq ($(CONSOLE),uart)
$(error Unknown CONSOLE '$(CONSOLE)'; use usb_cdc or uart)
endif

################################################################################
# CYW43439 radio (polling architecture). NET=lwip below adds lwIP on top,
# still within the same polling model -- see docs/RELEASE.md and
# lib/liblwip/sysdepend/pico_rp2040/.
################################################################################

ifeq ($(WIFI),cyw43)

CYW43_DRIVER_PATH := $(PICO_SDK_PATH)/lib/cyw43-driver
CYW43_PICO_PATH := $(PICO_SDK_PATH)/src/rp2_common/pico_cyw43_driver
WIFIDIR := ../lib/libwifi/sysdepend/pico_rp2040

ifeq ($(wildcard $(CYW43_DRIVER_PATH)/src/cyw43_ll.c),)
$(error CYW43 driver not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif

WIFI_LOCAL_SRCS := cyw43_utk.c cyw43_utk_kernel.c cyw43_utk_compat.c
WIFI_LOCAL_OBJS := $(addprefix ./mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/,$(WIFI_LOCAL_SRCS:.c=.o))

CYW43_OBJS := ./mtkernel_3/cyw43/cyw43_ll.o \
              ./mtkernel_3/cyw43/cyw43_ctrl.o \
              ./mtkernel_3/cyw43/cyw43_stats.o \
              ./mtkernel_3/cyw43/cyw43_bus_pio_spi.o \
              ./mtkernel_3/cyw43/pio.o \
              ./mtkernel_3/cyw43/dma.o \
              ./mtkernel_3/cyw43/gpio.o

WIFI_INCLUDES := -I"$(WIFIDIR)" \
                 -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
                 -I"$(CYW43_DRIVER_PATH)/src" \
                 -I"$(CYW43_DRIVER_PATH)/firmware" \
                 -I"$(CYW43_PICO_PATH)/include" \
                 -I"$(PICO_SDK_PATH)/src/boards/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                 -I"$(PICO_SDK_PATH)/src/common/hardware_claim/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_util/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_pio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_dma/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_clocks/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include"

CFLAGS += -DTM_WIFI_CYW43=1 -DCYW43_LWIP=$(if $(filter lwip,$(NET)),1,0) -DCYW43_ENABLE_BLUETOOTH=0 \
          -DCYW43_USE_OTP_MAC=1 -DPICO_CYW43_LOGGING_ENABLED=0 \
          -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 -DPICO_BUILD=1 \
          -DPICO_NO_HARDWARE=0 -DRASPBERRYPI_PICO_W=1 \
          -DCYW43_DEFAULT_PIN_WL_REG_ON=23u \
          -DCYW43_DEFAULT_PIN_WL_DATA_OUT=24u \
          -DCYW43_DEFAULT_PIN_WL_DATA_IN=24u \
          -DCYW43_DEFAULT_PIN_WL_HOST_WAKE=24u \
          -DCYW43_DEFAULT_PIN_WL_CLOCK=29u \
          -DCYW43_DEFAULT_PIN_WL_CS=25u \
          -DNDEBUG -ffunction-sections -fdata-sections $(WIFI_INCLUDES)
LFLAGS += -Wl,--gc-sections

OBJS += $(WIFI_LOCAL_OBJS) $(CYW43_OBJS)
C_DEPS += $(WIFI_LOCAL_OBJS:.o=.d) $(CYW43_OBJS:.o=.d)

mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/%.o: $(WIFIDIR)/%.c
	@echo 'Building Wi-Fi port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ll.o: $(CYW43_DRIVER_PATH)/src/cyw43_ll.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ctrl.o: $(CYW43_DRIVER_PATH)/src/cyw43_ctrl.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_stats.o: $(CYW43_DRIVER_PATH)/src/cyw43_stats.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_bus_pio_spi.o: $(CYW43_PICO_PATH)/cyw43_bus_pio_spi.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/pio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_pio/pio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/dma.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_dma/dma.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/gpio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

ifeq ($(NET),lwip)

LWIPDIR := $(PICO_SDK_PATH)/lib/lwip/src
LIBLWIP_PORT := ../lib/liblwip/sysdepend/pico_rp2040

ifeq ($(wildcard $(LWIPDIR)/core/init.c),)
$(error lwIP not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif

# COREFILES + CORE4FILES (IPv4 only) + the one NETIFFILES entry we need, per
# $(PICO_SDK_PATH)/lib/lwip/src/Filelists.mk. altcp*.c is included because
# apps/mqtt/mqtt.c (MQTT=1, below) is written against the altcp API -- used
# here in its default plain-TCP passthrough mode (LWIP_ALTCP=0), not TLS.
LWIP_CORE_SRCS := core/init.c core/def.c core/dns.c core/inet_chksum.c \
                   core/ip.c core/mem.c core/memp.c core/netif.c core/pbuf.c \
                   core/raw.c core/stats.c core/sys.c \
                   core/altcp.c core/altcp_alloc.c core/altcp_tcp.c \
                   core/tcp.c core/tcp_in.c core/tcp_out.c core/timeouts.c \
                   core/udp.c \
                   core/ipv4/acd.c core/ipv4/autoip.c core/ipv4/dhcp.c \
                   core/ipv4/etharp.c core/ipv4/icmp.c core/ipv4/igmp.c \
                   core/ipv4/ip4_frag.c core/ipv4/ip4.c core/ipv4/ip4_addr.c \
                   netif/ethernet.c
LWIP_CORE_OBJS := $(addprefix ./mtkernel_3/lwip/,$(LWIP_CORE_SRCS:.c=.o))

LWIP_PORT_OBJS := ./mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/sys_arch.o

CYW43_LWIP_OBJS := ./mtkernel_3/cyw43/cyw43_lwip.o

LWIP_INCLUDES := -I"$(LIBLWIP_PORT)/include" \
                 -I"$(LIBLWIP_PORT)" \
                 -I"$(LWIPDIR)/include"

CFLAGS += -DTM_NET_LWIP=1 $(LWIP_INCLUDES)

OBJS += $(LWIP_CORE_OBJS) $(LWIP_PORT_OBJS) $(CYW43_LWIP_OBJS)
C_DEPS += $(LWIP_CORE_OBJS:.o=.d) $(LWIP_PORT_OBJS:.o=.d) $(CYW43_LWIP_OBJS:.o=.d)

mtkernel_3/lwip/%.o: $(LWIPDIR)/%.c
	@echo 'Building lwIP file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/%.o: ../lib/liblwip/sysdepend/pico_rp2040/%.c
	@echo 'Building lwIP port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_lwip.o: $(CYW43_DRIVER_PATH)/src/cyw43_lwip.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

ifeq ($(MQTT),1)

MQTT_CORE_OBJS := ./mtkernel_3/lwip/apps/mqtt/mqtt.o
MQTT_PORT_OBJS := ./mtkernel_3/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.o

CFLAGS += -DTM_MQTT=1

OBJS += $(MQTT_CORE_OBJS) $(MQTT_PORT_OBJS)
C_DEPS += $(MQTT_CORE_OBJS:.o=.d) $(MQTT_PORT_OBJS:.o=.d)

mtkernel_3/lwip/apps/mqtt/mqtt.o: $(LWIPDIR)/apps/mqtt/mqtt.c
	@echo 'Building lwIP file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

endif

endif

endif

include mtkernel_3/lib/libtm/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/lib/libtm/sysdepend/no_device/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/subdir.mk
include mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/core/armv6m/subdir.mk
