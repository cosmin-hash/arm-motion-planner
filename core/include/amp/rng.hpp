/// @file rng.hpp
/// @brief Portable, seedable random number generator.
///
/// Why not std::uniform_real_distribution? The C++ standard fixes the output of
/// std::mt19937_64 bit-for-bit, but NOT the algorithm the distributions use to turn those
/// bits into doubles. MSVC and libstdc++ produce different sequences from the same seed.
/// Determinism is a requirement here (same seed + scenario => identical plan on every
/// platform), so we convert raw engine output to doubles ourselves.
#pragma once

#include <cstdint>
#include <random>

namespace amp {

/// Deterministic random number generator built on std::mt19937_64.
class Rng {
public:
    /// Create a generator; equal seeds always produce equal sequences on every platform.
    explicit Rng(std::uint64_t seed) : engine_(seed) {}

    /// Uniform double in [0, 1), using the top 53 bits of one engine draw.
    [[nodiscard]] double uniform01() {
        constexpr double kScale = 1.0 / 9007199254740992.0;  // 2^-53
        return static_cast<double>(engine_() >> 11) * kScale;
    }

    /// Uniform double in [lo, hi).
    [[nodiscard]] double uniform(double lo, double hi) { return lo + (hi - lo) * uniform01(); }

    /// Uniform integer in [0, n). Uses rejection to avoid modulo bias. @pre n > 0
    [[nodiscard]] std::uint64_t below(std::uint64_t n) {
        const std::uint64_t limit = UINT64_MAX - (UINT64_MAX % n);
        std::uint64_t x = engine_();
        while (x >= limit) {
            x = engine_();
        }
        return x % n;
    }

private:
    std::mt19937_64 engine_;
};

}  // namespace amp
