/*
 *
 * pi-fan-ctrl.c, oleg.komaristov@gmail.com 08/2021, MIT
 *
 * The only supported platform is Raspberry Pi. Requires bcm2835 library from
 * http://www.airspayce.com/mikem/bcm2835/index.html.
 * Requires root privelegies.
 *
 * Note: You can't use integrated audio and hardware PWM at the same time.
 *       On Ubuntu - disable audio by adding `dtparam=audio=off` to `/boot/firmware/usercfg.txt`.
 *
 * To build: `gcc -Wall pi-fan-ctrl.c -lbcm2835 -Ofast -o pi-fan-ctrl`.
 *
 * This app uses ideas and some solutions from the pi_fan_hwpwm.c by Allan Alwyn.
 * Link: https://gist.github.com/alwynallan/1c13096c4cd675f38405702e89e0c536.
 */

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <bcm2835.h>

#ifndef MAX
#define MAX(a, b) (a > b ? a : b)
#endif

#ifndef MIN
#define MIN(a, b) (a < b ? a : b)
#endif

typedef uint8_t pwmChannel;
typedef uint32_t pwmData;

const struct tagFanSettings {
	double	min_pwm_level;			// minimum level when the fan does not stop (in percentages 0.0-1.0)
	uint64_t start_impulse_duration;	// time in microseconds fan will be switched to full speed for the start
} FanSettings = { 0.3, 5000 };

const struct tagTempSettings {
	double on;
	double off;
	double high;
} TempSettings = { 55.0, 45.0, 70.0 };

typedef struct {
	pwmChannel channel;
	uint32_t clock_divisor;
	pwmData range;
} pwmSettings;
const pwmSettings PWMDefaults = { 0, BCM2835_PWM_CLOCK_DIVIDER_1, 1024 };

static const int8_t UnknownFanState = -1;
static const double UnknownTemp = -1e5;
static const pwmData UnknownLevel = 0xFFFFFFFF;

typedef enum {
	none = 0,
	initialization,
	running,
	finalization
} runMode;

typedef enum {
	off = 0,
	on = 1
} fanState;

typedef struct {
	runMode state;
	pid_t pid;

	fanState fan_state;
	double cpu_temp;

	pwmSettings *pwm_settings;
	pwmData pwm_level;
} appState;

static const char *ProcessIdFile = "/run/pi-fan-ctrl.pid";
static const char *StateFile = "/run/pi-fan-ctrl.state";

typedef int exst;
static const exst ok = 0;
static const exst err_state = -255;

typedef uint64_t sleep_interval;
static const sleep_interval Second = 1000000;
static const sleep_interval MinimumSleep = Second;
static const sleep_interval MaximumSleep = Second * 4lu;

extern exst check_pwm_channel(pwmChannel channel);
extern appState *initialize_service(pwmChannel channel);
extern exst file_write(appState *app, const char *fname, const char *data);
extern exst init_pwm(appState *app);
extern exst cleanup_service(appState *app);
extern exst measure_temp(appState *app);
extern exst update_pwm_range(appState *app);

extern void print_usage();
extern void error(char *fmt, ...);
extern void fatal(char *fmt, ...);

typedef struct {
	double value;
	sleep_interval interval;
} measurement;

extern sleep_interval next_sleep_interval(appState *app, sleep_interval current, measurement *measurements, size_t count);

// Do not use directly. It is added only for errors and signals handling proposes.
void *system_state = NULL;

int main(int argc, char *argv[]) {
	int opt;
	char buf[100];

	appState *state = NULL;
	_Bool verbose = 0;
	pwmChannel channel = PWMDefaults.channel;

	while ((opt = getopt(argc, argv, "c:v")) != -1) {
		switch (opt) {
		case 'c':
			channel = atoi(optarg);
			if (ok != check_pwm_channel(channel)) fatal("Channel %d not supported on this device", channel);
			break;
		case 'v':
			verbose = 1;
			break;
		default:
			print_usage();
			exit(EXIT_FAILURE);
		}
	}
	if(optind != argc) {
		error("optind=%d argc=%d Unrecognized parameter %s", optind, argc, argv[optind]);
		print_usage();
		exit(EXIT_FAILURE);
	}

	if (NULL == (state = initialize_service(channel))) fatal("Can't finish initialization");
	sprintf(buf, "%d\n", state->pid);
	if (ok != file_write(state, ProcessIdFile, buf)) fatal("Can't set pid to %s", ProcessIdFile);
	if (ok != init_pwm(state)) fatal("Unable initialize pwm at channel %d", state->pwm_settings->channel);
	state->state = running;

	static const size_t buffer_length = 5;
	measurement values_buffer_1[buffer_length], values_buffer_2[buffer_length];
	measurement *active_buffer = values_buffer_1;
	size_t buffer_offset = 0;

	sleep_interval duration = MinimumSleep;
	while (running == state->state) {
		if (ok != measure_temp(state) || ok != update_pwm_range(state)) continue;

		if (buffer_length == buffer_offset + 1) {
			measurement *inactive = (active_buffer == values_buffer_1) ? values_buffer_2 : values_buffer_1;
			memcpy(inactive, ((void *)active_buffer + sizeof(measurement)), sizeof(measurement) * buffer_offset);
			active_buffer = inactive;
		}
		else {
			buffer_offset++;
		}

		measurement *record = &(active_buffer[buffer_offset]);
		record->value = state->cpu_temp;
			record->interval = duration;
		duration = next_sleep_interval(state, duration, active_buffer, buffer_offset);

		sprintf(buf, "%.2f, %.1f\n", state->cpu_temp, (double)state->pwm_level/(double)state->pwm_settings->range*100.0);
		file_write(state, StateFile, buf);
		if(verbose) fputs(buf, stdout);

		usleep(duration);
	}

	exit(EXIT_SUCCESS);
}

#define LOG_AND_RETURN(__CODE__, ...) do { \
		error(__VA_ARGS__); \
		return __CODE__; \
} while(0)

exst check_pwm_channel(pwmChannel channel) {
	FILE *file;
	static const size_t max_len = 55 + 1;
	const char raspberry_pi_4b[] = "Raspberry Pi 4 Model B";
	const char cpu_info[] = "/proc/cpuinfo";
	char buffer[max_len + 1];

	if (0 == channel) return ok;
	if (1 < channel) return -2;

	if (NULL == (file = fopen(cpu_info, "r"))) LOG_AND_RETURN(-1, "Unable to open %s file for reading", cpu_info);
	fseek(file, -max_len, SEEK_END);
	size_t readen = fread(buffer, max_len, 1, file);
	buffer[readen] = '\0';

	char *last_newline = strrchr(buffer, '\n');
	char *last_line = last_newline + 1;
	int result = (NULL != strstr(last_line, raspberry_pi_4b)) ? ok : -1;
	fclose(file);

	return result;
}

void on_system_signal(int signum) {
	error("%d signal received", signum);
	cleanup_service((appState *)system_state);
	exit(signum);
}

exst subscribe_to_system_signals(void) {
	const int signals[] = { SIGHUP, SIGINT, SIGQUIT, SIGILL, SIGTRAP, SIGABRT, SIGBUS, SIGFPE, SIGSEGV, SIGPIPE, SIGTERM };
	const size_t count = sizeof(signals) / sizeof(int);

	for( size_t index = 0; index < count; ++index ) {
		signal(signals[index], on_system_signal);
	}
	return ok;
}

appState *initialize_service(pwmChannel channel) {
	appState *state = NULL;
	pwmSettings *settings = NULL;

	if (ok != subscribe_to_system_signals()) LOG_AND_RETURN(NULL, "Unable subscribe to system signals");

	settings = malloc(sizeof(pwmSettings));
	if (!settings) LOG_AND_RETURN(NULL, "Not enough memory");
	void *copied = memcpy(settings, &PWMDefaults, sizeof(PWMDefaults));
	if (NULL != copied && settings != copied) {
			free(settings);
			LOG_AND_RETURN(NULL, "Not enough memory");
	}
	settings->channel = channel;
	state = malloc(sizeof(appState));
	if (!state) LOG_AND_RETURN(NULL, "Not enough memory");
	memset(state, 0, sizeof(appState));
	state->pwm_settings = settings;
	state->state = initialization;
	state->pid = getpid();
	state->fan_state = UnknownFanState;
	state->cpu_temp = UnknownTemp;
	state->pwm_level = UnknownLevel;
	system_state = state;

	return state;
}

exst file_write(appState *app, const char *fname, const char *data) {
// https://opensource.com/article/19/4/interprocess-communication-linux-storage
	struct flock lock;
	int fd;
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;
	lock.l_start = 0;
	lock.l_len = 0;
	lock.l_pid = app->pid;

	if ((fd = open(fname, O_RDWR | O_CREAT, 0666)) < 0) LOG_AND_RETURN(-1, "failed to open %s for writing", fname);
	if (fcntl(fd, F_SETLK, &lock) < 0) {
	    close(fd);
	    LOG_AND_RETURN(-1, "fcntl failed to get lock on %s", fname);
    }

    exst ret_val = ok;
#define finalize do { \
    lock.l_type = F_UNLCK; \
    int unlocked = fcntl(fd, F_SETLK, &lock); \
    int closed = close(fd); \
    if (unlocked < 0) { \
        error("failed unlock %s (code %d)", fname, unlocked); \
        ret_val = -1; \
    } \
    if (closed < 0) { \
        error("failed to close '%s' (code %d)", fname, closed); \
        ret_val = -1; \
    } \
} while(0)
#define on_error(__code__, ...) do { \
    finalize; \
    error(__VA_ARGS__); \
    return __code__; \
} while(0)

	if (ftruncate(fd, 0) < 0) on_error(-1, "truncate failed to on %s", fname);
	const size_t data_length = strlen(data);
	if (-1 == write(fd, data, data_length)) on_error(-2, "failed to write %u bytes (error %d) to opened %s", data_length, errno, fname);
	finalize;
#undef on_error
#undef finalize
	return ret_val;
}

exst init_pwm(appState *app) {
	if (initialization != app->state) return -2;
	if (!bcm2835_init()) return -1;

	uint8_t pwm_pin = 0 == app->pwm_settings->channel ? 18 : 13;
	bcm2835_gpio_fsel(pwm_pin, BCM2835_GPIO_FSEL_ALT5);
	bcm2835_pwm_set_clock(app->pwm_settings->clock_divisor);
	bcm2835_pwm_set_mode(app->pwm_settings->channel, 1, 1);
	bcm2835_pwm_set_range(app->pwm_settings->channel, app->pwm_settings->range);
	bcm2835_pwm_set_data(app->pwm_settings->channel, 0);
	app->pwm_level = 0;
	app->fan_state = off;
	return ok;
}

exst start_fan(pwmSettings * const settings) {
		bcm2835_pwm_set_data(settings->channel, settings->range);
		usleep(FanSettings.start_impulse_duration);
		return ok;
}

exst stop_fan(pwmSettings * const settings) {
		return ok;
}

exst cleanup_service(appState *app) {
	if (!app) return ok;

	exst result = ok;
	if (running == app->state) {
		app->state = finalization;
		if (on == app->fan_state) stop_fan(app->pwm_settings);
		bcm2835_pwm_set_data(app->pwm_settings->channel, 0);
		bcm2835_pwm_set_mode(app->pwm_settings->channel, 1, 0);
		if (!bcm2835_close()) {
			error("Can't deinit bcm2835");
			result = -1;
		}

		if (0 != remove(ProcessIdFile)) error("Unable to remove PID file at %s", ProcessIdFile);
		if (0 != remove(StateFile)) error("Unable to remove state file at %s", StateFile);
	}

	app->state = none;
	return result;
}

exst measure_temp(appState *app) {
	FILE *file;
	int raw_temp;
	int read_status;

	if (NULL == (file = fopen("/sys/class/thermal/thermal_zone0/temp", "r"))) LOG_AND_RETURN(-1, "Can't open file '/sys/class/thermal/thermal_zone0/temp'");
	read_status = fscanf(file, "%d", &raw_temp);
	fclose(file);
	if (1 != read_status) LOG_AND_RETURN(-1, "Unable read temperature from '/sys/class/thermal/thermal_zone0/temp'");
	app->cpu_temp = (double)raw_temp / 1000.0;
	return ok;
}

pwmData calculate_level(pwmSettings * const settings, double temperature) {
	const double start_range = (double)settings->range * FanSettings.min_pwm_level;
	const double work_range = (double)settings->range - start_range;
	const double temp_range = TempSettings.high - TempSettings.off;
	const pwmData result = start_range + (temperature - TempSettings.off) / temp_range * work_range;
	return result;
}

exst update_pwm_range(appState *app) {
	if (UnknownTemp == app->cpu_temp) LOG_AND_RETURN(-1, "CPU temperature not set");

	const fanState fan = app->cpu_temp > TempSettings.on ? on : (app->cpu_temp < TempSettings.off ? off : app->fan_state);
	if (off == fan && fan == app->fan_state) return ok;

	pwmData level = calculate_level(app->pwm_settings, app->cpu_temp);

	#define apply_level(__level__) do { \
		if (running != app->state) return err_state; \
		bcm2835_pwm_set_data(app->pwm_settings->channel, __level__); \
		app->pwm_level = __level__; \
	} while(0)

	if (fan == app->fan_state) {
		const pwmData delta = abs((int64_t)level - (int64_t)app->pwm_level);
		const pwmData min_delta = (level > app->pwm_level ? 0.04 : 0.07) * (double)app->pwm_settings->range;
		if (delta < min_delta) return ok;

		apply_level(level);
		return ok;
	}
	if (running != app->state) return err_state;

	switch (fan) {
		case on:
			if (ok != start_fan(app->pwm_settings)) LOG_AND_RETURN(-2, "Unable start fan");
			break;
		case off:
			if (ok != stop_fan(app->pwm_settings)) LOG_AND_RETURN(-2, "Unable stop fan");
			level = 0;
			break;
	}

	apply_level(level);
	app->fan_state = fan;
	return ok;
}

void print_usage() {
	fprintf(stderr,
		"\n" \
		"Usage: sudo ./pi-fan-ctrl [OPTION]...\n" \
		"\n" \
		"		 -c <n> Use 'n' PWM channel for fan's PWM input, default 0 (GPIO 18).\n" \
		"										Only hardware PWM capable GPIO 18 and GPIO 13 (channel 1) are present on\n" \
		"										the RasPi 4B pin header, and only GPIO 18 can be used with\n" \
		"										the unmodified case fan.\n" \
		"		 -v		Verbose output\n" \
		"\n");
}

void error(char *fmt, ...) {
	char buf[128];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	fprintf(stderr, "%s\n", buf);
	fflush(stderr);
}

void fatal(char *fmt, ...) {
	char buf[128];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	fprintf(stderr, "%s\n", buf);
	fflush(stderr);

	cleanup_service((appState *)system_state);
	exit(EXIT_FAILURE);
}

sleep_interval next_sleep_interval(appState *app, sleep_interval current, measurement *measurements, size_t count) {
	static const size_t MinCount = 4;
	static const size_t EvalCount = 3;
	if (count < MinCount) return MinimumSleep;

	double *change_speeds = malloc((count - 1) * sizeof(double));
	if (!change_speeds) fatal("Not enough memory");

	double sum = 0.0;
	size_t eval_index = EvalCount + 1;
	for (size_t index = count -1; index > 1; --index) {
		measurement *current = &(measurements[index]);
		measurement *previous = &(measurements[index - 1]);

		change_speeds[index - 1] = (current->value - previous->value)/((double)current->interval / (double)Second);

		sum += change_speeds[index - 1];
		--eval_index;

		if (0 == eval_index) break;
		--eval_index;
	}
	double average = fabs(sum / (double)EvalCount);

	const double distance = MaximumSleep - MinimumSleep;
	sleep_interval result = current;
	if (average > 0.30) result -= average / (double)EvalCount * distance;
	if (average < 0.15) result += 0.1 * distance;

	result = MIN(result, MaximumSleep);
	result = MAX(result, MinimumSleep);
	return result;
}
