#include <rm_serial_protocol/nav_rx.hpp>
#include <cstdio>
int main(){
  rm_serial_protocol::NavRxData p;
  p.Navigation_x=1.25f;p.Navigation_y=-2.5f;p.Navigation_z=.75f;
  p.target_stance=3;
  p.nav_ready=1;
  p.super_cap_flag=2;
  p.if_guodong=0;
  p.if_stronger=4;
  p.sof1=-91;
  p.data_length1=4660;
  p.seq1=-7;
  p.crc8_1=86;
  p.cmd_id=9029;
  p.bt_status_data=-2;
  p.bt_remain_time=1234;
  p.bt_game_status=4;
  p.crc16_1=-12345;
  p.sof2=-91;
  p.data_length2=111;
  p.seq2=250;
  p.crc8_2=-1;
  p.cmd_id2=321;
  p.intention=5;
  p.start_position_x=-1200;
  p.start_position_y=2300;
  p.sender_id=107;
  p.crc16_2=-20000;
  p.crc16_3=30000;
  for(int i=0;i<49;++i){p.delta_x[i]=i-24;p.delta_y[i]=24-i;}
  const auto bytes=rm_serial_protocol::encodePayload(p);
  const rm_serial_protocol::Payload expected={0,0,160,63,0,0,32,192,0,0,64,63,3,1,2,0,4,165,52,18,249,86,69,35,254,210,4,4,199,207,165,111,0,250,255,65,1,5,80,251,252,8,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,107,0,224,177,48,117};
  if(bytes!=expected){std::fputs("all-field wire encoding mismatch\n",stderr);return 1;}
  const auto wire=rm_serial_protocol::encode(p);
  if(wire[0]!=0xA5 || wire[1]!=148 || wire.size()!=148) return 3;
  for(std::size_t i=0;i<expected.size();++i) if(wire[i+2]!=expected[i]) return 4;
  const auto short_wire=rm_serial_protocol::encode(p,19);
  if(short_wire[0]!=0xA5 || short_wire[1]!=19) return 5;
  for(std::size_t i=0;i<17;++i) if(short_wire[i+2]!=expected[i]) return 6;
  try { (void)rm_serial_protocol::encode(p,18); return 7; }
  catch(const std::invalid_argument&) {}
  const auto defaults=rm_serial_protocol::encode(rm_serial_protocol::NavRxData{});
  for(std::size_t i=0;i<defaults.size();++i){
    const uint8_t value=i==0?0xA5:(i==1?148:(i==14?3:((i==15||i==17)?1:0)));
    if(defaults[i]!=value){std::fputs("default packet mismatch\n",stderr);return 2;}
  }
  std::puts("PASS: A5 + total length + 146-byte MCU structure = 148 bytes; 19-byte control frame length verified, all field offsets, signed integers, arrays, little-endian floats, zero default speeds");
}
