#include "cv2x/cv2x_sidelink.hpp"
#include "fec/conv_encoder.hpp"
#include "fec/viterbi_decoder.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

// ─── BSM ─────────────────────────────────────────────────────────────────

std::vector<uint8_t> BSM::serialize() const {
    std::vector<uint8_t> out(13, 0);
    // vehicle_id — 4 bytes big endian
    out[0] = (vehicle_id >> 24) & 0xFF;
    out[1] = (vehicle_id >> 16) & 0xFF;
    out[2] = (vehicle_id >>  8) & 0xFF;
    out[3] = (vehicle_id      ) & 0xFF;
    // latitude — 4 bytes
    out[4] = (static_cast<uint32_t>(latitude) >> 24) & 0xFF;
    out[5] = (static_cast<uint32_t>(latitude) >> 16) & 0xFF;
    out[6] = (static_cast<uint32_t>(latitude) >>  8) & 0xFF;
    out[7] = (static_cast<uint32_t>(latitude)      ) & 0xFF;
    // longitude — 4 bytes (reuse latitude logic)
    uint32_t lon = static_cast<uint32_t>(longitude);
    out[8]  = (lon >> 24) & 0xFF;
    out[9]  = (lon >> 16) & 0xFF;
    out[10] = (lon >>  8) & 0xFF;
    out[11] = (lon      ) & 0xFF;
    // speed_cms — 2 bytes, heading — 2 bytes, brake — 1 byte
    // pack speed and heading into remaining byte via bit field
    out[12] = brake_status;
    // append speed and heading as separate bytes
    out.push_back((speed_cms >> 8) & 0xFF);
    out.push_back( speed_cms       & 0xFF);
    out.push_back((heading  >> 8) & 0xFF);
    out.push_back( heading        & 0xFF);
    return out;  // 17 bytes total
}

BSM BSM::deserialize(const std::vector<uint8_t>& b) {
    BSM bsm{};
    if (b.size() < 17) return bsm;
    bsm.vehicle_id  = (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16)
                    | (uint32_t(b[2]) <<  8) |  uint32_t(b[3]);
    bsm.latitude    = static_cast<int32_t>(
                       (uint32_t(b[4]) << 24) | (uint32_t(b[5]) << 16)
                     | (uint32_t(b[6]) <<  8) |  uint32_t(b[7]));
    bsm.longitude   = static_cast<int32_t>(
                       (uint32_t(b[8])  << 24) | (uint32_t(b[9])  << 16)
                     | (uint32_t(b[10]) <<  8) |  uint32_t(b[11]));
    bsm.brake_status = b[12];
    bsm.speed_cms   = (uint16_t(b[13]) << 8) | b[14];
    bsm.heading     = (uint16_t(b[15]) << 8) | b[16];
    return bsm;
}

std::string BSM::to_string() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "vehicle_id=0x" << std::hex << std::uppercase << vehicle_id
       << std::dec
       << "  lat="    << latitude  / 1e7
       << "  lon="    << longitude / 1e7
       << "  speed="  << speed_cms / 44.704f << " mph"
       << "  heading=" << heading * 0.0125f  << " deg"
       << "  brakes=0x" << std::hex << int(brake_status);
    return ss.str();
}

// ─── SCI ─────────────────────────────────────────────────────────────────

uint32_t SCI::pack() const {
    uint32_t w = 0;
    w |= (uint32_t(priority        & 0x07) << 29);
    w |= (uint32_t(resource_interval & 0x0F) << 25);
    w |= (uint32_t(mcs             & 0x1F) << 20);
    w |= (uint32_t(retx_index      & 0x03) << 18);
    w |= (uint32_t(resource_block  & 0x3FF) << 8);
    w |= (uint32_t(group_dst_id    & 0xFF)      );
    return w;
}

SCI SCI::unpack(uint32_t w) {
    SCI sci{};
    sci.priority          = (w >> 29) & 0x07;
    sci.resource_interval = (w >> 25) & 0x0F;
    sci.mcs               = (w >> 20) & 0x1F;
    sci.retx_index        = (w >> 18) & 0x03;
    sci.resource_block    = (w >>  8) & 0x3FF;
    sci.group_dst_id      =  w        & 0xFF;
    return sci;
}

std::string SCI::to_string() const {
    std::ostringstream ss;
    ss << "priority="   << int(priority)
       << "  mcs="      << int(mcs)
       << "  retx="     << int(retx_index)
       << "  rb="       << int(resource_block)
       << "  dst=0x"    << std::hex << int(group_dst_id);
    return ss.str();
}

// ─── PC5Frame ────────────────────────────────────────────────────────────

PC5Frame PC5Frame::encode(const BSM& bsm, const SCI& sci) {
    PC5Frame frame;
    frame.sci = sci;

    // serialize BSM to bytes then unpack to bits
    auto bytes = bsm.serialize();
    std::vector<uint8_t> bits;
    bits.reserve(bytes.size() * 8);
    for (uint8_t byte : bytes)
        for (int b = 7; b >= 0; --b)
            bits.push_back((byte >> b) & 1);

    // FEC encode — rate 1/2 K=7
    ConvolutionalEncoder enc(2, 7, {0133, 0171});
    frame.payload_bits = enc.encode(bits);

    return frame;
}

BSM PC5Frame::decode(const PC5Frame& frame) {
    // soft decision not available here — hard decode
    ViterbiDecoder dec(2, 7, {0133, 0171});
    auto decoded_bits = dec.decode_hard(frame.payload_bits);

    // pack bits back to bytes
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 7 < decoded_bits.size(); i += 8) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; ++b)
            byte |= (decoded_bits[i + b] << (7 - b));
        bytes.push_back(byte);
    }

    return BSM::deserialize(bytes);
}