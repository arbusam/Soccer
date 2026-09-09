#include "linux_kicker.h"

#include <cerrno>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <iterator>
#include <linux/gpio.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <system_error>
#include <unistd.h>

namespace hardware {
namespace {
std::string find_gpiochip() {
    // Match the same device-tree controllers used by Blinka. Pi 5's RP1 chip
    // number varies with kernel version, so do not hardcode gpiochip0 or gpiochip4.
    const std::filesystem::path root("/sys/bus/gpio/devices");
    std::string bcm_chip;
    if (std::filesystem::exists(root)) {
        for (const auto& entry : std::filesystem::directory_iterator(root)) {
            const auto name = entry.path().filename().string();
            if (name.rfind("gpiochip", 0) != 0) continue;
            std::ifstream file(entry.path() / "of_node/compatible", std::ios::binary);
            const std::string compatible{std::istreambuf_iterator<char>(file), {}};
            const auto device = "/dev/" + name;
            if (compatible.find("raspberrypi,rp1-gpio") != std::string::npos) return device;
            if (compatible.find("raspberrypi,bcm2835-gpio") != std::string::npos ||
                compatible.find("raspberrypi,bcm2711-gpio") != std::string::npos)
                bcm_chip = device;
        }
    }
    if (!bcm_chip.empty()) return bcm_chip;
    throw std::runtime_error("Cannot locate Raspberry Pi GPIO controller; specify kicker_gpiochip");
}
void checked_ioctl(int fd, unsigned long request, void* argument, const char* operation) {
    if (ioctl(fd, request, argument) < 0)
        throw std::system_error(errno, std::generic_category(), operation);
}
}
LinuxKickerOutput::LinuxKickerOutput(int bcm_pin, const std::string& gpiochip) {
    if (bcm_pin < 0 || bcm_pin > 27)
        throw std::invalid_argument("Kicker pin must be a BCM GPIO number in 0..27");
    const auto device = gpiochip.empty() ? find_gpiochip() : gpiochip;
    const int chip_fd = open(device.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) throw std::system_error(errno, std::generic_category(), "Open " + device);
    gpio_v2_line_request request{};
    request.offsets[0] = static_cast<unsigned>(bcm_pin);
    request.num_lines = 1;
    request.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
    std::strcpy(request.consumer, "soccer-kicker");
    try {
        checked_ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request, "Request kicker GPIO");
        line_fd_ = request.fd;
    } catch (...) {
        ::close(chip_fd);
        throw;
    }
    ::close(chip_fd);
}
LinuxKickerOutput::~LinuxKickerOutput() {
    try { close(); } catch (...) {
        // Explicit stop reports errors and can retry while the descriptor is owned.
        if (line_fd_ >= 0) ::close(line_fd_);
    }
}
void LinuxKickerOutput::high() {
    if (line_fd_ < 0) throw std::runtime_error("Kicker GPIO is closed");
    gpio_v2_line_config config{};
    config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    config.num_attrs = 1;
    config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    config.attrs[0].attr.values = 1;
    config.attrs[0].mask = 1;
    checked_ioctl(line_fd_, GPIO_V2_LINE_SET_CONFIG_IOCTL, &config, "Activate kicker GPIO");
    output_ = true;
}
void LinuxKickerOutput::idle() {
    if (line_fd_ < 0) return;
    std::exception_ptr error;
    if (output_) {
        gpio_v2_line_values values{};
        values.mask = 1;
        try {
            checked_ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values, "Set kicker GPIO low");
        } catch (...) { error = std::current_exception(); }
    }
    // Attempt pull-down even if setting low failed, so one error cannot skip cleanup.
    gpio_v2_line_config config{};
    config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
    try {
        checked_ioctl(line_fd_, GPIO_V2_LINE_SET_CONFIG_IOCTL, &config, "Restore kicker GPIO pull-down");
        output_ = false;
    } catch (...) { if (!error) error = std::current_exception(); }
    if (error) std::rethrow_exception(error);
}
void LinuxKickerOutput::close() {
    if (line_fd_ < 0) return;
    idle();
    ::close(line_fd_);
    line_fd_ = -1;
}
} // namespace hardware
