#include "kaanh/modbus.h"
#include <unistd.h> //用于usleep() 延迟挂起进程
#include <sys/timeb.h>		//用于打印时间
#include "kaanh/controlboard.h"
#include "kaanh.h"


//识别按键并发送指令
auto controlboard::send_command(uint16_t *buffer, aris::server::ControlServer &cs)->bool
{
	//buffer[0]和[1]分别对应华途示教器寄存器地址30001和30002,至少有一个二进制位为1
    uint32_t bit32 = (buffer[1] << 16) + buffer[0];
    int count_press = 0, index = -1;	//1的个数和1的下标
	for(int i = 0; i < 32; i++)
	{
		if((bit32 >> i) & 0x01)
		{
			count_press++;
			index = i;
		}
	}
	if(count_press > 1)
	{
		cout << "ERROR: Don't press 2 buttons at same time." <<endl;
		return 0;
	}
	
	// index的编号范围0~30，按键编号范围1~31，index=0代表按键1，index=30代表按键31
    char command[100];
    if (index <= 5)//"program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs"
	{
        cs.executeCmd(control_other[index].c_str());//"program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs"
        cout << control_other[index].c_str() << endl;
	}
    else if(index == 6)	//清除错误
    {
        cs.executeCmd(control_other[index].c_str());//cl
        cout << control_other[index].c_str() << endl;
        cs.executeCmd(control_other[index+1].c_str());//rc
        cout << control_other[index+1].c_str() << endl;
    }
    else if(index == 7)//ds
    {
        cs.executeCmd(control_other[index+1].c_str());//ds
        cout << control_other[index+1].c_str() << endl;
        sleep(1);
    }
    else if(index == 8)//ds,md en,rc
    {
        cs.executeCmd(control_other[index+1].c_str());//ds
        cout << control_other[index+1].c_str() << endl;
        cs.executeCmd(control_other[index+2].c_str());//md
        cout << control_other[index+2].c_str() << endl;
        cs.executeCmd(control_other[index+3].c_str());//en
        cout << control_other[index+3].c_str() << endl;
        cs.executeCmd(control_other[index+4].c_str());//rc
        cout << control_other[index+4].c_str() << endl;
        sleep(1);
    }
    else if(index >= 9 && index <= 20)//move j1~j5
	{
		//按键10至按键21，即右侧+-号
        if(joint_cart){
            cs.executeCmd(control_motor[index-9].c_str());
            cout << control_motor[index-9].c_str() << endl;
        }
        else
        {
            cs.executeCmd(control_pose[index-9].c_str());
            cout << control_pose[index-9].c_str() << endl;
        }
    }
    /*
	else if(index >= 29)
	{	
		//按键30和31，更改速度
		if(index == 29)	//按键30减速
		{
            //velocity = velocity - 1 >= 0 ? velocity - 1 : 0;
            velocity = g_vel_percent.load();
            string s_velocity = to_string(velocity);
            string cmd = "setvel --vel_percent="+s_velocity;
            cs.executeCmd(cmd);
            cout<<cmd<<endl;		
		}
		else//按键31加速
        {
            //velocity = velocity + 1 <= 100 ? velocity + 1 : 100;
            velocity = g_vel_percent.load();
            string s_velocity = to_string(velocity);
            string cmd = "setvel --vel_percent="+s_velocity;
            cs.executeCmd(cmd);
            cout<<cmd<<endl;
        }
	}
    */
    else if (index == 24)//this is Axis coordinates
	{
		joint_cart = true;
        cout<<"this is Axis coordinates"<<endl;

	}
    else if (index == 25)//this is Cartesian coordinates
	{
		joint_cart = false;
        cout<<"this is Cartesian coordinates"<<endl;
	}
    else if(index == 21)//change Base or TCP Cartesian coordinates
    {
        cor = !cor;
        if(cor)
        {
            for(int i=0; i<12; i++)
            {
                if (i < 6)
                    control_pose[i].replace(24, 1, "1");
                else
                    control_pose[i].replace(25, 1, "1");
            }
            cout<<"this is TCP Cartesian coordinates"<<endl;
        }
        else
        {
            for(int i=0; i<12; i++)
            {
                if (i < 6)
                    control_pose[i].replace(24, 1, "0");
                else
                    control_pose[i].replace(25, 1, "0");
            }
            cout<<"this is Base Cartesian coordinates"<<endl;
        }
		std::this_thread::sleep_for(std::chrono::seconds(1));
        cout << "cor:" <<cor <<std::endl;
    }
	else
	{
        cout << "Reserved" << endl;
    }
	return 1;
}
