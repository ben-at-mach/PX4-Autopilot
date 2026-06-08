/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file UbxHpe.cpp
 *
 * Minimal u-blox HPE receiver recorder.
 *
 * This module intentionally starts as a passive byte recorder for X20P/F9P-class
 * HPE receivers. Later revisions can add UBX-CFG-VALSET setup, message framing,
 * and typed HPE/RTK/status publishing without disturbing the generic PX4 GPS driver.
 */

#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/gps_dump.h>

using namespace time_literals;

class UbxHpe : public ModuleBase<UbxHpe>
{
public:
	UbxHpe(const char *device, uint32_t baudrate, const char *output_path, bool exclusive_log_file, bool publish_dump,
	       uint8_t dump_instance);
	~UbxHpe() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static UbxHpe *instantiate(int argc, char *argv[]);

	int print_status() override;
	void run() override;

private:
	static constexpr size_t kReadBufferSize = 512;
	static constexpr size_t kMinReadBytes = 32;
	static constexpr int kReadTimeoutMs = 50;
	static constexpr hrt_abstime kFsyncInterval = 5_s;
	static constexpr hrt_abstime kStaleDataInterval = 5_s;
	static constexpr const char *kDefaultOutputPath = "/fs/microsd/ubx_hpe.ubx";
	static constexpr const char *kIncrementOutputPattern = "/fs/microsd/ubx_hpe_%03u.ubx";
	static constexpr const char *kRealtimeOutputPattern = "/fs/microsd/ubx_hpe_%llu_%03lu.ubx";

	enum class LogNameMode : int32_t {
		Fixed = 0,
		Increment = 1,
		Realtime = 2,
	};

	static bool copy_string(char *dst, size_t dst_len, const char *src);
	static bool make_log_path(char *dst, size_t dst_len, LogNameMode mode, bool &exclusive);
	static bool make_increment_log_path(char *dst, size_t dst_len, bool &exclusive);
	static bool make_realtime_log_path(char *dst, size_t dst_len, bool &exclusive);
	static int32_t get_int_param(const char *name, int32_t default_value);

	int open_uart();
	int open_log_file();
	void close_files();
	void publish_gps_dump(const uint8_t *data, size_t len);
	void sync_log_file(bool force);

	char _device[64] {};
	char _output_path[96] {};
	uint32_t _baudrate {230400};
	bool _exclusive_log_file {false};
	bool _publish_dump {true};
	uint8_t _dump_instance {2};

	device::Serial _uart {};
	int _file_fd {-1};

	uint64_t _bytes_read {0};
	uint64_t _bytes_written {0};
	uint64_t _dump_messages {0};
	uint64_t _read_errors {0};
	uint64_t _write_errors {0};
	uint64_t _partial_writes {0};
	uint64_t _fsync_count {0};
	uint64_t _fsync_errors {0};
	size_t _max_read_chunk {0};
	size_t _max_write_chunk {0};
	hrt_abstime _start_time {0};
	hrt_abstime _last_byte_time {0};
	hrt_abstime _last_stale_warn_time {0};
	hrt_abstime _last_fsync_time {0};
	hrt_abstime _max_write_latency {0};
	hrt_abstime _max_fsync_latency {0};
	bool _file_dirty {false};

	perf_counter_t _cycle_perf {perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _read_perf {perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _write_perf {perf_alloc(PC_ELAPSED, MODULE_NAME": write")};
	perf_counter_t _fsync_perf {perf_alloc(PC_ELAPSED, MODULE_NAME": fsync")};
	perf_counter_t _read_error_perf {perf_alloc(PC_COUNT, MODULE_NAME": read errors")};
	perf_counter_t _write_error_perf {perf_alloc(PC_COUNT, MODULE_NAME": write errors")};
	perf_counter_t _partial_write_perf {perf_alloc(PC_COUNT, MODULE_NAME": partial writes")};
	perf_counter_t _fsync_error_perf {perf_alloc(PC_COUNT, MODULE_NAME": fsync errors")};

	gps_dump_s _dump {};
	uORB::Publication<gps_dump_s> _gps_dump_pub {ORB_ID(gps_dump)};
};

UbxHpe::UbxHpe(const char *device, uint32_t baudrate, const char *output_path, bool exclusive_log_file,
	       bool publish_dump, uint8_t dump_instance) :
	_baudrate(baudrate),
	_exclusive_log_file(exclusive_log_file),
	_publish_dump(publish_dump),
	_dump_instance(dump_instance)
{
	copy_string(_device, sizeof(_device), device);
	copy_string(_output_path, sizeof(_output_path), output_path);
}

UbxHpe::~UbxHpe()
{
	close_files();
	perf_free(_cycle_perf);
	perf_free(_read_perf);
	perf_free(_write_perf);
	perf_free(_fsync_perf);
	perf_free(_read_error_perf);
	perf_free(_write_error_perf);
	perf_free(_partial_write_perf);
	perf_free(_fsync_error_perf);
}

bool UbxHpe::copy_string(char *dst, size_t dst_len, const char *src)
{
	if (dst_len == 0 || src == nullptr) {
		return false;
	}

	strncpy(dst, src, dst_len - 1);
	dst[dst_len - 1] = '\0';
	return true;
}

int32_t UbxHpe::get_int_param(const char *name, int32_t default_value)
{
	int32_t value = default_value;
	const param_t param = param_find(name);

	if (param != PARAM_INVALID) {
		param_get(param, &value);
	}

	return value;
}

bool UbxHpe::make_realtime_log_path(char *dst, size_t dst_len, bool &exclusive)
{
	struct timespec ts {};

	if (px4_clock_gettime(CLOCK_REALTIME, &ts) != 0 || ts.tv_sec < 1704067200) {
		return false;
	}

	const int ret = snprintf(dst, dst_len, kRealtimeOutputPattern,
				 (unsigned long long)ts.tv_sec, (unsigned long)(ts.tv_nsec / 1000000));
	exclusive = true;
	return ret > 0 && static_cast<size_t>(ret) < dst_len;
}

bool UbxHpe::make_increment_log_path(char *dst, size_t dst_len, bool &exclusive)
{
	for (unsigned i = 0; i <= 999; ++i) {
		const int ret = snprintf(dst, dst_len, kIncrementOutputPattern, i);

		if (ret <= 0 || static_cast<size_t>(ret) >= dst_len) {
			return false;
		}

		if (::access(dst, F_OK) != 0) {
			exclusive = true;
			return true;
		}
	}

	return false;
}

bool UbxHpe::make_log_path(char *dst, size_t dst_len, LogNameMode mode, bool &exclusive)
{
	exclusive = false;

	if (mode == LogNameMode::Realtime) {
		if (make_realtime_log_path(dst, dst_len, exclusive)) {
			return true;
		}

		PX4_WARN("realtime invalid, falling back to incrementing filename");
		return make_increment_log_path(dst, dst_len, exclusive);
	}

	if (mode == LogNameMode::Increment) {
		return make_increment_log_path(dst, dst_len, exclusive);
	}

	return copy_string(dst, dst_len, kDefaultOutputPath);
}

int UbxHpe::open_uart()
{
	if (!_uart.setPort(_device)) {
		PX4_ERR("invalid serial device %s", _device);
		return PX4_ERROR;
	}

	if (!_uart.setBaudrate(_baudrate)) {
		PX4_ERR("failed to set baudrate %lu on %s", (unsigned long)_baudrate, _device);
		return PX4_ERROR;
	}

	if (!_uart.open()) {
		PX4_ERR("failed to open serial device %s", _device);
		return PX4_ERROR;
	}

	_uart.flush();

	return PX4_OK;
}

int UbxHpe::open_log_file()
{
	if (_output_path[0] == '\0') {
		return PX4_OK;
	}

	const int flags = O_WRONLY | O_CREAT | (_exclusive_log_file ? O_EXCL : O_TRUNC);
	_file_fd = ::open(_output_path, flags, 0666);

	if (_file_fd < 0) {
		PX4_ERR("failed to open log file %s", _output_path);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void UbxHpe::close_files()
{
	if (_file_fd >= 0) {
		sync_log_file(true);
		::close(_file_fd);
		_file_fd = -1;
	}

	if (_uart.isOpen()) {
		_uart.close();
	}
}

void UbxHpe::sync_log_file(bool force)
{
	if (_file_fd < 0 || (!_file_dirty && !force)) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!force && _last_fsync_time != 0 && now - _last_fsync_time < kFsyncInterval) {
		return;
	}

	const hrt_abstime fsync_start = hrt_absolute_time();
	perf_begin(_fsync_perf);
	const int ret = ::fsync(_file_fd);
	perf_end(_fsync_perf);
	const hrt_abstime fsync_latency = hrt_elapsed_time(&fsync_start);

	if (fsync_latency > _max_fsync_latency) {
		_max_fsync_latency = fsync_latency;
	}

	_last_fsync_time = hrt_absolute_time();

	if (ret == 0) {
		++_fsync_count;
		_file_dirty = false;

	} else {
		++_fsync_errors;
		perf_count(_fsync_error_perf);
	}
}

void UbxHpe::publish_gps_dump(const uint8_t *data, size_t len)
{
	if (!_publish_dump) {
		return;
	}

	_dump.instance = _dump_instance;
	_dump.device_id = 0;

	while (len > 0) {
		size_t write_len = len;
		const size_t capacity = sizeof(_dump.data) - _dump.len;

		if (write_len > capacity) {
			write_len = capacity;
		}

		memcpy(&_dump.data[_dump.len], data, write_len);
		_dump.len += write_len;
		data += write_len;
		len -= write_len;

		if (_dump.len >= sizeof(_dump.data)) {
			_dump.timestamp = hrt_absolute_time();
			_gps_dump_pub.publish(_dump);
			++_dump_messages;
			_dump.len = 0;
		}
	}
}

void UbxHpe::run()
{
	_start_time = hrt_absolute_time();
	_last_stale_warn_time = _start_time;

	if (open_uart() != PX4_OK || open_log_file() != PX4_OK) {
		close_files();
		return;
	}

	if (_publish_dump) {
		_gps_dump_pub.advertise();
	}

	PX4_INFO("recording %s at %lu baud%s%s%s", _device, (unsigned long)_baudrate,
		 _output_path[0] ? " to " : "", _output_path[0] ? _output_path : "",
		 _publish_dump ? " and gps_dump" : "");

	uint8_t buffer[kReadBufferSize];

	while (!should_exit()) {
		perf_begin(_cycle_perf);
		perf_begin(_read_perf);
		const ssize_t nread = _uart.readAtLeast(buffer, sizeof(buffer), kMinReadBytes, kReadTimeoutMs);
		perf_end(_read_perf);

		if (nread < 0) {
			++_read_errors;
			perf_count(_read_error_perf);
			perf_end(_cycle_perf);
			continue;
		}

		if (nread == 0) {
			const hrt_abstime now = hrt_absolute_time();

			if (_last_byte_time == 0) {
				if (now - _start_time > kStaleDataInterval && now - _last_stale_warn_time > kStaleDataInterval) {
					PX4_WARN("no receiver bytes yet on %s", _device);
					_last_stale_warn_time = now;
				}

			} else if (now - _last_byte_time > kStaleDataInterval && now - _last_stale_warn_time > kStaleDataInterval) {
				PX4_WARN("no receiver bytes for %.1fs", (double)(now - _last_byte_time) / 1e6);
				_last_stale_warn_time = now;
			}

			sync_log_file(false);
			perf_end(_cycle_perf);
			continue;
		}

		const size_t bytes_read = static_cast<size_t>(nread);
		_last_byte_time = hrt_absolute_time();
		_bytes_read += bytes_read;

		if (bytes_read > _max_read_chunk) {
			_max_read_chunk = bytes_read;
		}

		publish_gps_dump(buffer, bytes_read);

		if (_file_fd >= 0) {
			const hrt_abstime write_start = hrt_absolute_time();
			perf_begin(_write_perf);
			const ssize_t nwritten = ::write(_file_fd, buffer, bytes_read);
			perf_end(_write_perf);
			const hrt_abstime write_latency = hrt_elapsed_time(&write_start);

			if (write_latency > _max_write_latency) {
				_max_write_latency = write_latency;
			}

			if (nwritten > 0) {
				_bytes_written += static_cast<uint64_t>(nwritten);
				_file_dirty = true;

				if (static_cast<size_t>(nwritten) > _max_write_chunk) {
					_max_write_chunk = static_cast<size_t>(nwritten);
				}
			}

			if (nwritten != nread) {
				++_write_errors;
				perf_count(_write_error_perf);

				if (nwritten > 0) {
					++_partial_writes;
					perf_count(_partial_write_perf);
				}
			}
		}

		sync_log_file(false);
		perf_end(_cycle_perf);
	}

	if (_publish_dump && _dump.len > 0) {
		_dump.timestamp = hrt_absolute_time();
		_gps_dump_pub.publish(_dump);
		++_dump_messages;
		_dump.len = 0;
	}

	PX4_INFO("stopped, read=%llu written=%llu dump_msgs=%llu",
		 (unsigned long long)_bytes_read,
		 (unsigned long long)_bytes_written,
		 (unsigned long long)_dump_messages);
	close_files();
}

UbxHpe *UbxHpe::instantiate(int argc, char *argv[])
{
	const char *device = "/dev/ttyS3";
	char output_path[96] {};
	bool exclusive_log_file = false;
	uint32_t baudrate = 230400;
	bool publish_dump = get_int_param("UBX_HPE_DUMP", 1) != 0;
	const bool log_file = get_int_param("UBX_HPE_FILE", 1) != 0;
	int32_t dump_instance = get_int_param("UBX_HPE_DUMP_I", 2);
	int32_t log_name_mode = get_int_param("UBX_HPE_FNAME", static_cast<int32_t>(LogNameMode::Increment));

	if (log_name_mode < static_cast<int32_t>(LogNameMode::Fixed) ||
	    log_name_mode > static_cast<int32_t>(LogNameMode::Realtime)) {
		log_name_mode = static_cast<int32_t>(LogNameMode::Increment);
	}

	if (log_file
	    && !make_log_path(output_path, sizeof(output_path), static_cast<LogNameMode>(log_name_mode), exclusive_log_file)) {
		PX4_ERR("failed to create log path");
		return nullptr;
	}

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:o:x", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b': {
				int parsed_baudrate = 0;

				if (px4_get_parameter_value(myoptarg, parsed_baudrate) != 0) {
					PX4_ERR("baudrate parsing failed");
					return nullptr;
				}

				baudrate = static_cast<uint32_t>(parsed_baudrate);
			}
			break;

		case 'o':
			copy_string(output_path, sizeof(output_path), myoptarg);
			exclusive_log_file = false;
			break;

		case 'x':
			publish_dump = false;
			break;

		default:
			print_usage("unrecognized option");
			return nullptr;
		}
	}

	if (dump_instance < 0) {
		dump_instance = 0;

	} else if (dump_instance > UINT8_MAX) {
		dump_instance = UINT8_MAX;
	}

	UbxHpe *instance = new UbxHpe(device, baudrate, output_path, exclusive_log_file, publish_dump,
				      static_cast<uint8_t>(dump_instance));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int UbxHpe::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ubx_hpe",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SLOW_DRIVER,
				      PX4_STACK_ADJUSTED(2048),
				      &run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return PX4_OK;
}

int UbxHpe::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UbxHpe::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	const double runtime_s = (_start_time > 0 && now > _start_time) ? (double)(now - _start_time) / 1e6 : 0.0;
	const double read_rate = runtime_s > 0.0 ? (double)_bytes_read / runtime_s : 0.0;
	const double write_rate = runtime_s > 0.0 ? (double)_bytes_written / runtime_s : 0.0;
	const double dump_rate = runtime_s > 0.0 ? (double)_dump_messages / runtime_s : 0.0;

	PX4_INFO("device: %s", _device);
	PX4_INFO("baudrate: %lu", (unsigned long)_baudrate);
	PX4_INFO("uart open: %s", _uart.isOpen() ? "yes" : "no");
	PX4_INFO("output: %s%s", _output_path[0] ? _output_path : "disabled", _exclusive_log_file ? " (exclusive)" : "");
	PX4_INFO("gps_dump: %s instance=%u", _publish_dump ? "enabled" : "disabled", _dump_instance);
	PX4_INFO("runtime: %.1fs", runtime_s);
	PX4_INFO("bytes read: %llu", (unsigned long long)_bytes_read);
	PX4_INFO("bytes written: %llu", (unsigned long long)_bytes_written);
	PX4_INFO("rates: read %.1f B/s write %.1f B/s gps_dump %.1f msg/s", read_rate, write_rate, dump_rate);
	PX4_INFO("max chunks: read %u write %u", (unsigned)_max_read_chunk, (unsigned)_max_write_chunk);
	PX4_INFO("gps_dump messages: %llu", (unsigned long long)_dump_messages);
	PX4_INFO("read errors: %llu write errors: %llu partial writes: %llu",
		 (unsigned long long)_read_errors,
		 (unsigned long long)_write_errors,
		 (unsigned long long)_partial_writes);
	PX4_INFO("fsync: count=%llu errors=%llu max=%.3fms",
		 (unsigned long long)_fsync_count,
		 (unsigned long long)_fsync_errors,
		 (double)_max_fsync_latency / 1000.0);
	PX4_INFO("max write latency: %.3fms", (double)_max_write_latency / 1000.0);

	if (_last_byte_time > 0) {
		PX4_INFO("last byte: %.3fs ago", (double)(now - _last_byte_time) / 1e6);

	} else {
		PX4_INFO("last byte: never");
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_read_perf);
	perf_print_counter(_write_perf);
	perf_print_counter(_fsync_perf);
	perf_print_counter(_read_error_perf);
	perf_print_counter(_write_error_perf);
	perf_print_counter(_partial_write_perf);
	perf_print_counter(_fsync_error_perf);
	return PX4_OK;
}

int UbxHpe::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Passive raw UART recorder for u-blox HPE receivers such as ZED-X20P and ZED-F9P.

The first revision records receiver bytes to a `.ubx` file and publishes the same incoming bytes on
PX4's existing `gps_dump` uORB topic. It does not configure or parse the receiver yet.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ubx_hpe", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start passive UBX recording");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 230400, 9600, 4000000, "UART baudrate", true);
	PRINT_MODULE_USAGE_PARAM_STRING('o', kDefaultOutputPath, "<file>", "Override output .ubx path, empty disables file logging", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Disable gps_dump uORB publication", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ubx_hpe_main(int argc, char *argv[])
{
	return UbxHpe::main(argc, argv);
}
