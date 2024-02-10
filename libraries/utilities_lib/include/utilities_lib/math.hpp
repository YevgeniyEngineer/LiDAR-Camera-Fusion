#pragma once

#include <cmath>
#include <cstdint>

namespace utilities_lib
{
static constexpr inline std::int32_t constexprRound(const float value) noexcept
{
    return (value > 0.0F) ? static_cast<std::int32_t>(value + 0.5F) : static_cast<std::int32_t>(value - 0.5F);
}

static constexpr inline std::int64_t constexprRound(const double value) noexcept
{
    return (value > 0.0) ? static_cast<std::int64_t>(value + 0.5) : static_cast<std::int64_t>(value - 0.5);
}

static constexpr float atan2Approx(const float y, const float x) noexcept
{
    const float ax = std::fabs(x);
    const float ay = std::fabs(y);
    const float mx = std::max(ay, ax);
    const float mn = std::min(ay, ax);
    const float a = mn / mx;
    /* Minimax polynomial approximation to atan(a) on [0,1] */
    const float s = a * a;
    const float c = s * a;
    const float q = s * s;
    float r = 0.024840285F * q + 0.18681418F;
    const float t = -0.094097948F * q - 0.33213072F;
    r = r * s + t;
    r = r * c + a;
    /* Map to full circle */
    if (ay > ax)
    {
        r = 1.57079637F - r;
    }
    if (x < 0)
    {
        r = 3.14159274F - r;
    }
    if (y < 0)
    {
        r = -r;
    }
    return r;
}
} // namespace utilities_lib
