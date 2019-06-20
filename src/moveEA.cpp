#include<iostream>
//程序引用sris库的头文件
#include<aris.hpp>

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	
{
	/*创建std::unique_ptr实例*/
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	/*定义Ethercat控制器的xml文件
		phy_id指电机的序号，由两个电机，此处先完成0号电机的xml配置
		product_code、vendor_id等数据由控制器读取
		min_pos、max_pos与电缸 的行程有关，前者小于电缸的最小位置0mm，后
		     者大于电缸的最大形成100mm
		max_vel根据电缸的额定转速和行程来计算，即
		     3000（转速）*4（导程）/60/1000=0.2（单位m/s）
		min_vel与max_vel大小相等，方向相反
		max_acc按照经验定义为速度的10倍
		max_pos_following_error、max_vel_following_error由经验数据确定
		home_pos指电缸的初始位置，定义为0
		pos_factor指电缸在推进1米的情况下，控制器的电信号个数，通过查询电机
		     为23bit，则电机转动一圈的情况下电脉冲的次数是2^23=8388608个，电缸推
		     进一米需要转动250圈，则pos_factor=8388608*250=2097152000
		pos_offset是指电机在断电重启后电机的初始位置距离0点的偏差*/
	std::string xml_str =
		"<EthercatMotion phy_id=\"0\" product_code=\"0x60380007\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
		" min_pos=\"0.01\" max_pos=\"0.26\" max_vel=\"0.125\" min_vel=\"-0.125\""
		" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
		" home_pos=\"0\" pos_factor=\"-3355443200\" pos_offset=\"0.0\">"
		"	<SyncManagerPoolObject>"
		"		<SyncManager is_tx=\"false\"/>"
		"		<SyncManager is_tx=\"true\"/>"
		"		<SyncManager is_tx=\"false\">"
		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"		<SyncManager is_tx=\"true\">"
		"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"	</SyncManagerPoolObject>"
		"</EthercatMotion>";
	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
	return controller;
};

// 创建轨迹
struct MoveEAPParam
{
	double axis_begin_pos;
	double axis_pos;
	double axis_vel;
	double axis_acc;
	double axis_dec;
	bool abs;
};
class MoveEAP : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        auto c = target.controller;
		MoveEAPParam param;
        param.axis_begin_pos = 0.0;

		for (auto &p : params)
		{
			if (p.first == "pos")
			{
				param.axis_pos = std::stod(p.second);
			}
			else if (p.first == "vel")
			{	
				param.axis_vel = std::stod(p.second);
			}
			else if (p.first == "acc")
			{
				param.axis_acc = std::stod(p.second);
			}
			else if (p.first == "dec")
			{
				param.axis_dec = std::stod(p.second);
			}
			else if (p.first == "ab")
			{
				param.abs = std::stoi(p.second);
			}
		}

		target.param = param;
		std::string ret = "ok";
		target.ret = ret;

	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveEAPParam&>(target.param);
		// 访问主站 //
        auto controller = target.controller;
		// 第一个count，取各个电机的当前位置
		if (target.count == 1)
		{
			param.axis_begin_pos = controller->motionAtAbs(0).actualPos();
		}
        aris::Size total_count{ 1 };
		double p, v, a;
        aris::Size t_count;
		//绝对轨迹//
		if (param.abs)
		{
			//梯形轨迹规划函数moveAbsolute对输入的量(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000)都会取绝对值
			//然后规划出当前count的指令位置p，指令速度v，指令加速度a，以及本梯形轨迹的总count数t_count 
			aris::plan::moveAbsolute(target.count, param.axis_begin_pos, param.axis_pos, param.axis_vel / 1000, param.axis_acc / 1000 / 1000, param.axis_dec / 1000 / 1000, p, v, a, t_count);
			controller->motionAtAbs(0).setTargetPos(p);
			total_count = std::max(total_count, t_count);
		}
		//相对轨迹//
		else
		{
			aris::plan::moveAbsolute(target.count, param.axis_begin_pos, param.axis_begin_pos + param.axis_pos, param.axis_vel / 1000, param.axis_acc / 1000 / 1000, param.axis_dec /1000 / 1000, p, v, a, t_count);
			controller->motionAtAbs(0).setTargetPos(p);
			total_count = std::max(total_count, t_count);
		}

		// 每1000ms打印 目标位置、实际位置、实际速度、实际电流 //
		auto &cout = controller->mout();
		if (target.count % 1000 == 0)
		{
			cout << controller->motionAtAbs(0).targetPos() << "  " 
				<< controller->motionAtAbs(0).actualPos() << "  " 
				<< controller->motionAtAbs(0).actualVel() << "  " 
				<< controller->motionAtAbs(0).actualCur() << std::endl;
		}
		// log 目标位置、实际位置、实际速度、实际电流 //
		
		auto &lout = controller->lout();
		lout << controller->motionAtAbs(0).targetPos() << "  " 
			<< controller->motionAtAbs(0).actualPos() << "  " 
			<< controller->motionAtAbs(0).actualVel() << "  " 
			<< controller->motionAtAbs(0).actualCur() << "  ";
		lout << std::endl;
		// 返回total_count - target.count给aris实时核，值为-1，报错；值为0，结束；值大于0，继续执行下一个count
		return total_count - target.count;
	}

	auto virtual collectNrt(PlanTarget &target)->void {}

    explicit MoveEAP(const std::string &name = "MoveEAP_plan") :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveEAP\">"
			"	<GroupParam>"
			"		<Param name=\"begin_pos\" default=\"0.1\" abbreviation=\"b\"/>"
			"		<Param name=\"pos\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.02\"/>"
			"		<Param name=\"acc\" default=\"0.3\"/>"
			"		<Param name=\"dec\" default=\"-0.3\"/>"
			"		<Param name=\"ab\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}
};

// 将轨迹MoveEAP添加到轨迹规划池planPool中
auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

	plan_root->planPool().add<aris::plan::Enable>();
	plan_root->planPool().add<aris::plan::Disable>();
	plan_root->planPool().add<aris::plan::Mode>();
	plan_root->planPool().add<aris::plan::Show>();
	plan_root->planPool().add<aris::plan::Recover>();
	auto &rs = plan_root->planPool().add<aris::plan::Reset>();
	rs.command().findParam("pos")->setDefaultValue("{0.01}");
	plan_root->planPool().add<MoveEAP>();
	return plan_root;
}

// 主函数
int main(int argc, char *argv[])
{
	//创建Ethercat主站对象
    //aris::control::EthercatMaster mst;
	//自动扫描，连接从站
    //mst.scan();
    //std::cout<<mst.xmlString()<<std::endl;

	//cs代表成员函数的引用，aris是头文件，server是命名空间，ControlServer是结构体
    auto&cs = aris::server::ControlServer::instance();
    cs.resetController(createControllerRokaeXB4().release());
    cs.resetPlanRoot(createPlanRootRokaeXB4().release());

    std::cout<<"start controller server"<<std::endl;
	//启动线程
	cs.start();
	//getline是将输入的值赋值给command_in
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto id = cs.executeCmd(aris::core::Msg(command_in));
			std::cout << "command id:" << id << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}
}
