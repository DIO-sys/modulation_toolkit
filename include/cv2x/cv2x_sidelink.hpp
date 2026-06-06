#pragma once
#include <vector>
#include <cstdint>
#include <string>

// ─── Basic Safety Message ─────────────────────────────────────────────────
// Simplified SAE J2735 BSM — core fields only

struct BSM {
    uint32_t vehicle_id;   // 4 bytes
    int32_t  latitude;     // degrees * 1e7
    int32_t  longitude;    // degrees * 1e7
    uint16_t speed_cms;    // cm/s
    uint16_t heading;      // 0.0125 deg units (0-28799)
    uint8_t  brake_status; // bit field

    // serialize to bytes
    std::vector<uint8_t> serialize() const;

    // deserialize from bytes
    static BSM deserialize(const std::vector<uint8_t>& bytes);

    // human-readable print
    std::string to_string() const;
};

// ─── SCI Format 1 ────────────────────────────────────────────────────────
// 3GPP TS 36.212 sidelink control information
// 32 bits total

struct SCI {
    uint8_t  priority;           // 3 bits  — 0-7
    uint8_t  resource_interval; // 4 bits  — reservation interval
    uint8_t  mcs;               // 5 bits  — modulation and coding scheme
    uint8_t  retx_index;        // 2 bits  — retransmission index
    uint16_t resource_block;    // 10 bits — starting resource block
    uint8_t  group_dst_id;      // 8 bits  — destination group ID

    // pack to 32-bit word
    uint32_t pack() const;

    // unpack from 32-bit word
    static SCI unpack(uint32_t word);

    std::string to_string() const;
};

// ─── PC5 frame ───────────────────────────────────────────────────────────
// SCI header + FEC-encoded BSM payload

struct PC5Frame {
    SCI                  sci;
    std::vector<uint8_t> payload_bits;  // encoded BSM bits

    // full encode: BSM -> serialize -> FEC encode -> frame
    static PC5Frame encode(const BSM& bsm, const SCI& sci);

    // full decode: frame -> FEC decode -> deserialize -> BSM
    static BSM decode(const PC5Frame& frame);
};