#include <openarmx/robstride_motor/rs_motor_control.hpp>
#include <iostream>
int main(){namespace r=openarmx::robstride_motor; for(int i=1;i<=8;i++){r::Motor m(r::MotorType::RS00,i,i);auto p=r::CanPacketEncoder::create_state_request_command(m);std::cout<<std::hex<<p.send_can_id;for(auto b:p.data)std::cout<<" "<<int(b);auto s=r::CanPacketDecoder::parse_motor_state_data(m,{0,0,0,0,0,0,0,0},0x02000100);std::cout<<std::dec<<" angle0="<<s.angle<<"\n";}}
