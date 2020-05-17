#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "aris.hpp"
#include "modbus.h"
#define print_time false	//�Ƿ���Ҫ��ӡʱ��


class controlboard
{
public:
    auto send_command(uint16_t *buffer, aris::server::ControlServer &cs)->bool;//����ָ��

private:
	bool cor = 0;				//Ĭ�Ϲ�������ϵ
    bool joint_cart = true;     //Ĭ�Ϲؽ�����ϵ
	int velocity = 1;			//Ĭ�ϼӼ���ֵΪ1

	//����ָ�
	std::string control_other[13] = { "program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs","cl","rc","ds","ds","md","en","rc" };

	//�ؽ�ָ�
	std::string control_motor[12] = { "j1 --direction=-1","j1 --direction=1",
							"j2 --direction=-1","j2 --direction=1",
							"j3 --direction=-1","j3 --direction=1",
							"j4 --direction=-1","j4 --direction=1",
							"j5 --direction=-1","j5 --direction=1",
							"j6 --direction=-1","j6 --direction=1" };

    //�ѿ�������ϵָ�
	std::string control_pose[12] = { "jx --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jx --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jy --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jy --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jz --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jz --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrx --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrx --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jry --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jry --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrz --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrz --direction=1 --cor=0 --tool=tool0 --wobj=wobj0" };
};


#endif // CONTROLBOARD_H
