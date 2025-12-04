// ILED.h
#pragma once
#include <cstdint>

// Simple RGB color struct
struct RGBColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Pattern types
enum class LEDPattern {
    Solid,
    Blink,
    Breathe,
    FastBlink,
    SlowBlink
};

// Pattern speed (optional granularity)
enum class PatternSpeed {
    Slow,
    Medium,
    Fast
};

// Interface for LED control
class ILED {
public:
    virtual ~ILED() = default;

    // Basic LED control
    virtual void color(const RGBColor& c) = 0;
    virtual void color(uint8_t R, uint8_t G, uint8_t B) = 0;
    virtual void control(bool enable) = 0;
    virtual void reset() = 0;

    // Pattern control
    virtual void setPattern(LEDPattern pattern,
                            const RGBColor& c,
                            PatternSpeed speed = PatternSpeed::Medium) = 0;
};