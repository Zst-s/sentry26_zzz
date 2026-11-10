#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace rm_serial_protocol {
#pragma pack(push, 1)
// Exact field order of the supplied MCU __packed NAV_Rx_Data_t.
struct NavRxData {
  float Navigation_x=0, Navigation_y=0, Navigation_z=0;
  uint8_t target_stance=3, nav_ready=1, super_cap_flag=0;
  uint8_t if_guodong=1; // 1=raise, 0=fold
  uint8_t if_stronger=0;
  int8_t sof1=0;
  int16_t data_length1=0;
  int8_t seq1=0, crc8_1=0;
  int16_t cmd_id=0;
  int8_t bt_status_data=0;
  int16_t bt_remain_time=0;
  int8_t bt_game_status=0;
  int16_t crc16_1=0;
  int8_t sof2=0;
  int16_t data_length2=0;
  uint8_t seq2=0;
  int8_t crc8_2=0;
  int16_t cmd_id2=0;
  uint8_t intention=0;
  int16_t start_position_x=0, start_position_y=0;
  int8_t delta_x[49]={}, delta_y[49]={};
  int16_t sender_id=0, crc16_2=0, crc16_3=0;
};
#pragma pack(pop)
inline constexpr std::size_t kPayloadBytes=146, kPayloadControlBytes=17;
inline constexpr std::size_t kFrameBytes=148, kControlBytes=19;
inline constexpr uint8_t kHeader=0xA5;
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(NavRxData)==kPayloadBytes && alignof(NavRxData)==1);
#define RM_OFFSET(field, offset) static_assert(offsetof(NavRxData, field)==offset, #field " offset")
RM_OFFSET(Navigation_x,0); RM_OFFSET(Navigation_y,4); RM_OFFSET(Navigation_z,8);
RM_OFFSET(target_stance,12); RM_OFFSET(nav_ready,13); RM_OFFSET(super_cap_flag,14);
RM_OFFSET(if_guodong,15); RM_OFFSET(if_stronger,16);
RM_OFFSET(sof1,17); RM_OFFSET(data_length1,18); RM_OFFSET(seq1,20); RM_OFFSET(crc8_1,21);
RM_OFFSET(cmd_id,22); RM_OFFSET(bt_status_data,24); RM_OFFSET(bt_remain_time,25);
RM_OFFSET(bt_game_status,27); RM_OFFSET(crc16_1,28);
RM_OFFSET(sof2,30); RM_OFFSET(data_length2,31); RM_OFFSET(seq2,33); RM_OFFSET(crc8_2,34);
RM_OFFSET(cmd_id2,35); RM_OFFSET(intention,37);
RM_OFFSET(start_position_x,38); RM_OFFSET(start_position_y,40);
RM_OFFSET(delta_x,42); RM_OFFSET(delta_y,91); RM_OFFSET(sender_id,140);
RM_OFFSET(crc16_2,142); RM_OFFSET(crc16_3,144);
#undef RM_OFFSET
using Payload=std::array<uint8_t,kPayloadBytes>;
using Frame=std::array<uint8_t,kFrameBytes>;
inline void put16(Payload& out, std::size_t pos, int16_t value) {
  const auto bits=static_cast<uint16_t>(value);
  out[pos]=static_cast<uint8_t>(bits); out[pos+1]=static_cast<uint8_t>(bits>>8);
}
inline void putFloat(Payload& out, std::size_t pos, float value) {
  uint32_t bits; std::memcpy(&bits,&value,sizeof(bits));
  for(std::size_t i=0;i<4;++i) out[pos+i]=static_cast<uint8_t>(bits>>(8*i));
}
// Payload uses the supplied packed layout. Wire prefix is 0xA5 followed by total frame byte count.
// CRC values are copied as supplied; this function does not invent CRCs.
inline Payload encodePayload(const NavRxData& v) {
  Payload out{};
  putFloat(out,0,v.Navigation_x); putFloat(out,4,v.Navigation_y); putFloat(out,8,v.Navigation_z);
#define BYTE(f) out[offsetof(NavRxData,f)]=static_cast<uint8_t>(v.f)
#define WORD(f) put16(out,offsetof(NavRxData,f),v.f)
  BYTE(target_stance); BYTE(nav_ready); BYTE(super_cap_flag); BYTE(if_guodong); BYTE(if_stronger);
  BYTE(sof1); WORD(data_length1); BYTE(seq1); BYTE(crc8_1); WORD(cmd_id);
  BYTE(bt_status_data); WORD(bt_remain_time); BYTE(bt_game_status); WORD(crc16_1);
  BYTE(sof2); WORD(data_length2); BYTE(seq2); BYTE(crc8_2); WORD(cmd_id2); BYTE(intention);
  WORD(start_position_x); WORD(start_position_y);
  for(std::size_t i=0;i<49;++i){out[42+i]=static_cast<uint8_t>(v.delta_x[i]);out[91+i]=static_cast<uint8_t>(v.delta_y[i]);}
  WORD(sender_id); WORD(crc16_2); WORD(crc16_3);
#undef BYTE
#undef WORD
  return out;
}
// MCU checks byte0=0xA5 and byte1=total bytes, then parses payload at byte2.
inline Frame encode(const NavRxData& v, std::size_t frame_bytes=kFrameBytes) {
  if(frame_bytes!=kFrameBytes && frame_bytes!=kControlBytes)
    throw std::invalid_argument("frame size must be 148 or 19");
  Frame out{};
  out[0]=kHeader;
  out[1]=static_cast<uint8_t>(frame_bytes);
  const auto payload=encodePayload(v);
  for(std::size_t i=0;i<payload.size();++i) out[i+2]=payload[i];
  return out;
}
static_assert(kFrameBytes <= 255);
static_assert(kControlBytes == 2 + kPayloadControlBytes);
static_assert(kFrameBytes == 2 + sizeof(NavRxData));
static_assert(2 + offsetof(NavRxData,if_guodong) == 17);
} // namespace rm_serial_protocol
