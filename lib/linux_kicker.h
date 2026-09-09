#pragma once

#include <string>

namespace hardware {
// Injection point for offline pulse tests. Only the kicker thread uses these
// methods during normal operation; stop() joins it before closing the output.
class KickerOutput {
public:
    virtual ~KickerOutput() = default;
    virtual void high() = 0;
    virtual void idle() = 0;
    virtual void close() = 0;
};

class LinuxKickerOutput final : public KickerOutput {
public:
    explicit LinuxKickerOutput(int bcm_pin, const std::string& gpiochip = "");
    ~LinuxKickerOutput() override;
    LinuxKickerOutput(const LinuxKickerOutput&) = delete;
    LinuxKickerOutput& operator=(const LinuxKickerOutput&) = delete;
    void high() override;
    void idle() override;
    void close() override;
private:
    int line_fd_ = -1;
    bool output_ = false;
};
} // namespace hardware
