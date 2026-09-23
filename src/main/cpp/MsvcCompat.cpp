// Compatibility shims for MSVC CRT ABI differences with REVLib / GTest on Windows
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cstdint>
#include <cstring>

extern "C" {

void _Thrd_sleep_for(unsigned long ms) noexcept {
    Sleep(ms);
}

const void* __std_find_end_1(
    const void* _First1, const void* _Last1,
    const void* _First2, size_t _Count2) noexcept {
    const auto* f1 = static_cast<const uint8_t*>(_First1);
    const auto* l1 = static_cast<const uint8_t*>(_Last1);
    const auto* f2 = static_cast<const uint8_t*>(_First2);
    if (_Count2 == 0) return _Last1;
    if (l1 - f1 < static_cast<ptrdiff_t>(_Count2)) return nullptr;
    const uint8_t* res = nullptr;
    for (const auto* cur = f1; cur + _Count2 <= l1; ++cur) {
        if (std::memcmp(cur, f2, _Count2) == 0) {
            res = cur;
        }
    }
    return res;
}

const void* __std_search_1(
    const void* _First1, const void* _Last1,
    const void* _First2, size_t _Count2) noexcept {
    const auto* f1 = static_cast<const uint8_t*>(_First1);
    const auto* l1 = static_cast<const uint8_t*>(_Last1);
    const auto* f2 = static_cast<const uint8_t*>(_First2);
    if (_Count2 == 0) return _First1;
    if (l1 - f1 < static_cast<ptrdiff_t>(_Count2)) return _Last1;
    for (const auto* cur = f1; cur + _Count2 <= l1; ++cur) {
        if (std::memcmp(cur, f2, _Count2) == 0) {
            return cur;
        }
    }
    return _Last1;
}

const void* __std_search_2(
    const void* _First1, const void* _Last1,
    const void* _First2, size_t _Count2) noexcept {
    const auto* f1 = static_cast<const uint16_t*>(_First1);
    const auto* l1 = static_cast<const uint16_t*>(_Last1);
    const auto* f2 = static_cast<const uint16_t*>(_First2);
    if (_Count2 == 0) return _First1;
    if (l1 - f1 < static_cast<ptrdiff_t>(_Count2)) return _Last1;
    for (const auto* cur = f1; cur + _Count2 <= l1; ++cur) {
        if (std::memcmp(cur, f2, _Count2 * 2) == 0) {
            return cur;
        }
    }
    return _Last1;
}

}
#endif
