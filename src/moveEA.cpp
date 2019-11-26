#include<iostream>
#include<aris.hpp>
#include"moveEA.h"

using namespace std;
//调用aris库中的plan模块
using namespace aris::plan;

//创建ethercat主站控制器controller，并根据xml文件添加从站信息
auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
	double pos_offset = 0;	//在零位时，为500count
	double pos_factor = 8388608.0/360;	//23位编码器
	double max_pos = 10000.0;
	double min_pos = -10000.0;
	double max_vel = 90;	//1度/s
	double max_acc = 360;	//5度/s2
	std::string xml_str =
		"<EthercatMotion phy_id=\"" + std::to_string(6) + "\" product_code=\"0x6038000D\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
		" min_pos=\"" + std::to_string(min_pos) + "\" max_pos=\"" + std::to_string(max_pos) + "\" max_vel=\"" + std::to_string(max_vel) + "\" min_vel=\"" + std::to_string(-max_vel) + "\""
		" max_acc=\"" + std::to_string(max_acc) + "\" min_acc=\"" + std::to_string(-max_acc) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
		" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor) + "\" pos_offset=\"" + std::to_string(pos_offset) + "\">"
		"	<SyncManagerPoolObject>"
		"		<SyncManager is_tx=\"false\"/>"
		"		<SyncManager is_tx=\"true\"/>"
		"		<SyncManager is_tx=\"false\">"
		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"		<SyncManager is_tx=\"true\">"
		"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
		"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
		"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
		"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
		"			</Pdo>"
		"		</SyncManager>"
		"	</SyncManagerPoolObject>"
		"</EthercatMotion>";

	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
	return controller;
};


// 单关节正弦往复轨迹 //
struct MoveJSParam
{
	double j1;
	double time;
	uint32_t timenum;
};
auto MoveJS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveJSParam param;

	param.j1 = 0.0;
	param.time = 0.0;
	param.timenum = 0;

	for (auto &p : params)
	{
		if (p.first == "j1")
		{
			if (p.second == "current_pos")
			{
				param.j1 = target.controller->motionPool()[0].actualPos();
			}
			else
			{
				param.j1 = std::stod(p.second);
			}

		}
		else if (p.first == "time")
		{
			param.time = std::stod(p.second);
		}
		else if (p.first == "timenum")
		{
			param.timenum = std::stoi(p.second);
		}
	}
	target.param = param;

	std::fill(target.mot_options.begin(), target.mot_options.end(),
		Plan::USE_TARGET_POS);

}
auto MoveJS::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveJSParam&>(target.param);
	auto time = static_cast<int32_t>(param.time * 1000);
	auto totaltime = static_cast<int32_t>(param.timenum * time);
	static double begin_pjs;
	static double step_pjs;

	if ((1 <= target.count) && (target.count <= time / 2))
	{
		// 获取当前起始点位置 //
		if (target.count == 1)
		{
			begin_pjs = target.controller->motionPool()[0].actualPos();
			step_pjs = target.controller->motionPool()[0].actualPos();
		}
		step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*target.count / time)) / 2;
		target.controller->motionPool().at(0).setTargetPos(step_pjs);
	}
	else if ((time / 2 < target.count) && (target.count <= totaltime - time / 2))
	{
		// 获取当前起始点位置 //
		if (target.count == time / 2 + 1)
		{
			begin_pjs = target.controller->motionPool()[0].actualPos();
			step_pjs = target.controller->motionPool()[0].actualPos();
		}

		step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(target.count - time / 2) / time)) / 2;
		target.controller->motionPool().at(0).setTargetPos(step_pjs);
	}
	else if ((totaltime - time / 2 < target.count) && (target.count <= totaltime))
	{
		// 获取当前起始点位置 //
		if (target.count == totaltime - time / 2 + 1)
		{
			begin_pjs = target.controller->motionPool()[0].actualPos();
			step_pjs = target.controller->motionPool()[0].actualPos();
		}
		step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(target.count - totaltime + time / 2) / time)) / 2;
		target.controller->motionPool().at(0).setTargetPos(step_pjs);
	}

	if (target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = target.controller;

	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 100 == 0)
	{
		cout << "pos"  << ":" << controller->motionAtAbs(0).actualPos() << "  ";
		cout << "vel"  << ":" << controller->motionAtAbs(0).actualVel() << "  ";
		cout << "cur"  << ":" << controller->motionAtAbs(0).actualCur() << "  ";

		cout << std::endl;
	}

	// log 电流 //
	auto &lout = controller->lout();
	lout << controller->motionAtAbs(0).targetPos() << ",";
	lout << controller->motionAtAbs(0).actualPos() << ",";
	lout << controller->motionAtAbs(0).actualVel() << ",";
	lout << controller->motionAtAbs(0).actualCur() << ",";
	lout << std::endl;

	return totaltime - target.count;
}
auto MoveJS::collectNrt(PlanTarget &target)->void {}
MoveJS::MoveJS(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"mvjs\">"
		"	<GroupParam>"
		"		<Param name=\"j1\" default=\"current_pos\"/>"
		"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
		"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
		"	</GroupParam>"
		"</Command>");
}


// 将创建的轨迹添加到轨迹规划池planPool中 //
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
	plan_root->planPool().add<MoveJS>();
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
