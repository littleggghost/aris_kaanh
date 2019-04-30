#include <algorithm>
#include"kaanh.h"
#include<array>


using namespace aris::dynamic;
using namespace aris::plan;

extern double fce_data[buffer_length];
extern int data_num, data_num_send;
extern std::atomic_int which_di;
extern std::atomic_bool is_automatic;

namespace kaanh
{
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{

		std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
		

		std::string xml_str =
			"<EthercatSlave phy_id=\"6\" product_code=\"0x00013D6F\""
			" vendor_id=\"0x00000009\" revision_num=\"0x01\" dc_assign_activate=\"0x300\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1601\" is_tx=\"false\">"
			"				<PdoEntry name=\"Output_Instruction\" index=\"0x7010\" subindex=\"0x01\" size=\"16\"/>"
			"				<PdoEntry name=\"Output_Para1\" index=\"0x7010\" subindex=\"0x02\" size=\"16\"/>"
			"				<PdoEntry name=\"Output_Para2\" index=\"0x7010\" subindex=\"0x03\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A03\" is_tx=\"true\">"
            "				<PdoEntry name=\"Real_Input_DataNo\" index=\"0x6030\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"Real_Input_Fx\" index=\"0x6030\" subindex=\"0x01\" size=\"32\"/>"
            "				<PdoEntry name=\"Real_Input_Fy\" index=\"0x6030\" subindex=\"0x02\" size=\"32\"/>"
            "				<PdoEntry name=\"Real_Input_Fz\" index=\"0x6030\" subindex=\"0x03\" size=\"32\"/>"
            "				<PdoEntry name=\"Real_Input_Mx\" index=\"0x6030\" subindex=\"0x04\" size=\"32\"/>"
            "				<PdoEntry name=\"Real_Input_My\" index=\"0x6030\" subindex=\"0x05\" size=\"32\"/>"
            "				<PdoEntry name=\"Real_Input_Mz\" index=\"0x6030\" subindex=\"0x06\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatSlave>";

        controller->slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);

		return controller;
	};
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", aris::core::Matrix(PI));

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");
		auto &p2 = model->partPool().add<Part>("L2");
		auto &p3 = model->partPool().add<Part>("L3");
		auto &p4 = model->partPool().add<Part>("L4");
		auto &p5 = model->partPool().add<Part>("L5");
		auto &p6 = model->partPool().add<Part>("L6");

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };		//x方向加上0.1
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		/*
		makI.prtPm();
		makI.pm();

		double pq_ee_i[]{ 0.5, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };
		auto &tool1 = p6.markerPool().add<Marker>("tool1", pm_ee_i);
		tool1.prtPm();
		*/

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
			p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
			p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
			p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
			p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
			j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		inverse_kinematic.setWhichRoot(8);

		return model;
	}


	// 末端四元数xyz方向余弦轨迹；速度前馈//
	struct MoveXParam
	{
		double x, y, z;
		double time;
	};
	auto MoveX::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveXParam param ={0.0,0.0,0.0,0.0};
			for (auto &p : params)
			{
				if (p.first == "x")
				{
					param.x = std::stod(p.second);
				}
				else if (p.first == "y")
				{
					param.y = std::stod(p.second);
				}
				else if (p.first == "z")
				{
					param.z = std::stod(p.second);
				}
				else if (p.first == "time")
				{
					param.time = std::stod(p.second);
				}
			}
			target.param = param;

			target.option |=
				//用于使用模型轨迹驱动电机//
				Plan::USE_TARGET_POS |
                //Plan::USE_VEL_OFFSET |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START|
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR|
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveX::executeRT(PlanTarget &target)->int
		{ 
			auto &ee = target.model->generalMotionPool().at(0);
			auto &param = std::any_cast<MoveXParam&>(target.param);

			auto time = static_cast<int>(param.time*1000);
			static double begin_pq[7];
			if (target.count == 1)
			{
				ee.getMpq(begin_pq);
			}
			double pq2[7];
			double pqv[7] = {0.0, 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0};
			ee.getMpq(pq2);
			pq2[0] = begin_pq[0] + param.x*(1 - std::cos(2 * PI*target.count / time)) / 2;
			pq2[1] = begin_pq[1] + param.y*(1 - std::cos(2 * PI*target.count / time)) / 2;
			pq2[2] = begin_pq[2] + param.z*(1 - std::cos(2 * PI*target.count / time)) / 2;
			ee.setMpq(pq2);
			//速度前馈//
            pqv[0] = 1000 * param.x*(PI / time)*std::sin(2 * PI*target.count / time);
            pqv[1] = 1000 * param.y*(PI / time)*std::sin(2 * PI*target.count / time);
            pqv[2] = 1000 * param.z*(PI / time)*std::sin(2 * PI*target.count / time);
            ee.setMvq(pqv);

			if (!target.model->solverPool().at(0).kinPos())return -1;
            target.model->solverPool().at(0).kinVel();

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				 cout <<"cur:"<< controller->motionAtAbs(0).actualCur() <<"  "<< controller->motionAtAbs(1).actualCur() << std::endl;
			}
			
			// log 电流 //
			auto &lout = controller->lout();
			lout << controller->motionAtAbs(0).actualCur() << "  " << controller->motionAtAbs(1).actualCur() << std::endl;

			return time-target.count;
		}
	auto MoveX::collectNrt(PlanTarget &target)->void {}
	MoveX::MoveX(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveX\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"x\">"
			"			<Param name=\"x\" default=\"0.1\"/>"
			"			<Param name=\"y\" default=\"0.1\"/>"
			"			<Param name=\"z\" default=\"0.1\"/>"
			"		</UniqueParam>"
			"		<Param name=\"time\" default=\"1.0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 单关节正弦往复轨迹 //
	struct MoveJSParam
	{
		double j[6];
		double time;
		uint32_t timenum;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			//MoveJSParam param = {{0.0,0.0,0.0,0.0,0.0,0.0},0.0,0};
			MoveJSParam param;
			for (Size i = 0; i < 6; i++)
			{
				param.j[i] = 0.0;
			}
			param.time = 0.0;
			param.timenum = 0;	

			param.joint_active_vec.clear();
			param.joint_active_vec.resize(target.model->motionPool().size(), true);

			for (auto &p : params)
			{
				if (p.first == "j1")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[0] = false;
						param.j[0] = target.model->motionPool()[0].mp();
					}
					else
					{
						param.joint_active_vec[0] = true;
						param.j[0] = std::stod(p.second);
					}
							
				}
				else if (p.first == "j2")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[1] = false;
						param.j[1] = target.model->motionPool()[1].mp();
					}
					else
					{
						param.joint_active_vec[1] = true;
						param.j[1] = std::stod(p.second);
					}
				}
				else if (p.first == "j3")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[2] = false;
						param.j[2] = target.model->motionPool()[2].mp();
					}
					else
					{
						param.joint_active_vec[2] = true;
						param.j[2] = std::stod(p.second);
					}
				}
				else if (p.first == "j4")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[3] = false;
						param.j[3] = target.model->motionPool()[3].mp();
					}
					else
					{
						param.joint_active_vec[3] = true;
						param.j[3] = std::stod(p.second);
					}
				}
				else if (p.first == "j5")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[4] = false;
						param.j[4] = target.model->motionPool()[4].mp();
					}
					else
					{
						param.joint_active_vec[4] = true;
						param.j[4] = std::stod(p.second);
					}
				}
				else if (p.first == "j6")
				{
					if (p.second == "current_pos")
					{
						param.joint_active_vec[5] = false;
						param.j[5] = target.model->motionPool()[5].mp();
					}
					else
					{
						param.joint_active_vec[5] = true;
						param.j[5] = std::stod(p.second);
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

			target.option |=
				Plan::USE_TARGET_POS |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJS::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJSParam&>(target.param);
			auto time = static_cast<int32_t>(param.time * 1000);
			auto totaltime = static_cast<int32_t>(param.timenum * time);
			static double begin_pjs[6];
			static double step_pjs[6];
			
			if ((1 <= target.count) && (target.count <= time / 2))
			{
				// 获取当前起始点位置 //
				if (target.count == 1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] + param.j[i] * (1 - std::cos(2 * PI*target.count / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}
			else if ((time / 2 < target.count) && (target.count <= totaltime - time/2))
			{
				// 获取当前起始点位置 //
				if (target.count == time / 2+1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] - 2*param.j[i] * (1 - std::cos(2 * PI*(target.count-time/2) / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}

			}
			else if ((totaltime - time / 2 < target.count) && (target.count <= totaltime))
			{
				// 获取当前起始点位置 //
				if (target.count == totaltime - time / 2 + 1)
				{
					for (Size i = 0; i < param.joint_active_vec.size(); ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					step_pjs[i] = begin_pjs[i] - param.j[i] * (1 - std::cos(2 * PI*(target.count - totaltime + time / 2) / time)) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}

			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i+1 << ":" << controller->motionAtAbs(i).actualPos() << "  " ;
					cout << "vel" << i+1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i+1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}		
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;
			
			return totaltime - target.count;
		}
	auto MoveJS::collectNrt(PlanTarget &target)->void {}
	MoveJS::MoveJS(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJS\">"
			"	<GroupParam>"
			"		<Param name=\"j1\" default=\"current_pos\"/>"
			"		<Param name=\"j2\" default=\"current_pos\"/>"
			"		<Param name=\"j3\" default=\"current_pos\"/>"
			"		<Param name=\"j4\" default=\"current_pos\"/>"
			"		<Param name=\"j5\" default=\"current_pos\"/>"
			"		<Param name=\"j6\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 任意关节正弦往复轨迹 //
	struct MoveJSNParam
	{
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_time_vec;
		std::vector<bool> joint_active_vec;
		uint32_t timenum;
	};
	auto MoveJSN::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			MoveJSNParam param;
			auto c = target.controller;
			param.axis_pos_vec.clear();
			param.axis_pos_vec.resize(target.model->motionPool().size(), 0.0);

			param.axis_time_vec.clear();
			param.axis_time_vec.resize(target.model->motionPool().size(), 1.0);

			param.joint_active_vec.clear();
			param.joint_active_vec.resize(target.model->motionPool().size(), true);

			param.timenum = 0;
			for (auto &p : params)
			{
				if (p.first == "pos")
				{
					auto pos = target.model->calculator().calculateExpression(p.second);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						//超阈值保护//
						if (param.axis_pos_vec[i] > 1.0)
						{
							param.axis_pos_vec[i] = 1.0;
						}
						if (param.axis_pos_vec[i] < -1.0)
						{
							param.axis_pos_vec[i] = -1.0;
						}
						if (param.axis_pos_vec[i] >= 0)
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
						}
						else
						{
							param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
						}
					}

				}
				else if (p.first == "time")
				{
					auto t = target.model->calculator().calculateExpression(p.second);
					if (t.size() == 1)
					{
						param.axis_time_vec.resize(param.axis_time_vec.size(), t.toDouble());
					}
					else if (t.size() == param.axis_time_vec.size())
					{
						param.axis_time_vec.assign(t.begin(), t.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_time_vec.size(); ++i)
					{
						//超阈值保护，机器人单关节运动频率不超过5Hz//
						if (param.axis_time_vec[i] < 0.2)
						{
							param.axis_time_vec[i] = 0.2;
						}
					}
				}
				else if (p.first == "timenum")
				{
					param.timenum = std::stoi(p.second);
				}
			}
			target.param = param;

			target.option |=
				Plan::USE_TARGET_POS |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJSN::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveJSNParam&>(target.param);
			static int32_t time[6];
			static int32_t totaltime[6];
			static int32_t totaltime_max = 0;
			for (Size i = 0; i < 6; i++)
			{
				time[i] = static_cast<uint32_t>(param.axis_time_vec[i] * 1000);
				totaltime[i] = static_cast<uint32_t>(param.timenum * time[i]);
				if (totaltime[i] > totaltime_max)
				{
					totaltime_max = totaltime[i];
				}
			}

			static double begin_pjs[6];
			static double step_pjs[6];

			for (Size i = 0; i < param.axis_time_vec.size(); i++)
			{
				if ((1 <= target.count) && (target.count <= time[i] / 2))
				{
					// 获取当前起始点位置 //
					if (target.count == 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] + param.axis_pos_vec[i] * (1 - std::cos(2 * PI*target.count / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
				else if ((time[i] / 2 < target.count) && (target.count <= totaltime[i] - time[i] / 2))
				{
					// 获取当前起始点位置 //
					if (target.count == time[i] / 2 + 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] - 2 * param.axis_pos_vec[i] * (1 - std::cos(2 * PI*(target.count - time[i] / 2) / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);

				}
				else if ((totaltime[i] - time[i] / 2 < target.count) && (target.count <= totaltime[i]))
				{
					// 获取当前起始点位置 //
					if (target.count == totaltime[i] - time[i] / 2 + 1)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
					step_pjs[i] = begin_pjs[i] - param.axis_pos_vec[i] * (1 - std::cos(2 * PI*(target.count - totaltime[i] + time[i] / 2) / time[i])) / 2;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			}

			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = target.controller;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return totaltime_max - target.count;
		}
	auto MoveJSN::collectNrt(PlanTarget &target)->void {}
	MoveJSN::MoveJSN(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJSN\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"{0.1,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"p\"/>"
			"		<Param name=\"time\" default=\"{1.0,1.0,1.0,1.0,1.0,1.0}\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}
		

	// 单关节相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct MoveJRParam
	{
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		MoveJRParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		target.option |=
//				Plan::USE_TARGET_POS |
			Plan::USE_VEL_OFFSET |
#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
			Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;

		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					param.begin_joint_pos_vec[i] = controller->motionAtAbs(i).actualPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_joint_pos_vec[i], param.begin_joint_pos_vec[i]+param.joint_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
				controller->motionAtAbs(i).setTargetPos(p);
				controller->motionAtAbs(i).setTargetVel(v*1000);
				total_count = std::max(total_count, t_count);

				target.model->motionPool().at(i).setMp(p);
			}
		}

		//controller与模型同步，保证3D仿真模型同步显示
		if (!target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
                cout << "mp" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
                cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).targetPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<UniqueParam default=\"check_none\">"
			"			<Param name=\"check_all\"/>"
			"			<Param name=\"check_none\"/>"
			"			<GroupParam>"
			"				<UniqueParam default=\"check_pos\">"
			"					<Param name=\"check_pos\"/>"
			"					<Param name=\"not_check_pos\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_pos_max\">"
			"							<Param name=\"check_pos_max\"/>"
			"							<Param name=\"not_check_pos_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_min\">"
			"							<Param name=\"check_pos_min\"/>"
			"							<Param name=\"not_check_pos_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous\">"
			"							<Param name=\"check_pos_continuous\"/>"
			"							<Param name=\"not_check_pos_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_at_start\">"
			"							<Param name=\"check_pos_continuous_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order\">"
			"							<Param name=\"check_pos_continuous_second_order\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
			"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_following_error\">"
			"							<Param name=\"check_pos_following_error\"/>"
			"							<Param name=\"not_check_pos_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"				<UniqueParam default=\"check_vel\">"
			"					<Param name=\"check_vel\"/>"
			"					<Param name=\"not_check_vel\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_vel_max\">"
			"							<Param name=\"check_vel_max\"/>"
			"							<Param name=\"not_check_vel_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_min\">"
			"							<Param name=\"check_vel_min\"/>"
			"							<Param name=\"not_check_vel_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous\">"
			"							<Param name=\"check_vel_continuous\"/>"
			"							<Param name=\"not_check_vel_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous_at_start\">"
			"							<Param name=\"check_vel_continuous_at_start\"/>"
			"							<Param name=\"not_check_vel_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_following_error\">"
			"							<Param name=\"check_vel_following_error\"/>"
			"							<Param name=\"not_check_vel_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}
	

	// 梯形轨迹2测试--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct MoveTTTParam
	{
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		std::vector<double> begin_axis_vel_vec;
		std::vector<double> begin_axis_acc_vec;
		std::vector<double> begin_axis_dec_vec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveTTT::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveTTTParam param;

			for (auto cmd_param : params)
			{
				if (cmd_param.first == "all")
				{
					param.joint_active_vec.resize(c->motionPool().size(), true);
				}
				else if (cmd_param.first == "none")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
				}
				else if (cmd_param.first == "motion_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
				}
				else if (cmd_param.first == "physical_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
				}
				else if (cmd_param.first == "slave_id")
				{
					param.joint_active_vec.resize(c->motionPool().size(), false);
					param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
				}
				else if (cmd_param.first == "pos")
				{
					aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
					if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
					else
					{
						param.joint_pos_vec.resize(mat.size());
						std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
					}
				}
				else if (cmd_param.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(cmd_param.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
					}
					else if (v.size() == c->motionPool().size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (cmd_param.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(cmd_param.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
					}
					else if (a.size() == c->motionPool().size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (cmd_param.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(cmd_param.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
					}
					else if (d.size() == c->motionPool().size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}

			param.begin_joint_pos_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_vel_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_acc_vec.resize(c->motionPool().size(), 0.0);
			param.begin_axis_dec_vec.resize(c->motionPool().size(), 0.0);


			target.param = param;

			target.option |=
				Plan::USE_TARGET_POS |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveTTT::executeRT(PlanTarget &target)->int
		{
			auto &param = std::any_cast<MoveTTTParam&>(target.param);
			auto controller = target.controller;

			if (target.count == 1)
			{
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					if (param.joint_active_vec[i])
					{
						//param.begin_joint_pos_vec[i] = controller->motionPool()[i].actualPos();
						param.begin_joint_pos_vec[i] = target.model->motionPool().at(i).mp();
					}
				}
			}

			aris::Size total_count{ 1 };
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					double p, v, a;
					aris::Size t_count;
					auto result = aris::plan::moveAbsolute2(param.begin_joint_pos_vec[i], param.begin_axis_vel_vec[i], param.begin_axis_acc_vec[i], param.joint_pos_vec[i], 0.0, 0.0, param.axis_vel_vec[i], param.axis_acc_vec[i], param.axis_acc_vec[i], 1e-3, 1e-10, p, v, a, t_count);
					//controller->motionAtAbs(i).setTargetPos(p);
					target.model->motionPool().at(i).setMp(p);
                    total_count = result;
                    //total_count = std::max(total_count, t_count);

					param.begin_joint_pos_vec[i] = p;
					param.begin_axis_vel_vec[i] = v;
					param.begin_axis_acc_vec[i] = a;
				}
			}

            if (!target.model->solverPool().at(1).kinPos())return -1;
			   
			// 打印电流 //
			auto &cout = controller->mout();
            if (target.count % 1000 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
                    if (param.joint_active_vec[i])
                    {
                        cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
                        cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
                        cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
                    }
				}
				cout << std::endl;
			}

			auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
			fwd.cptJacobi();

			auto &lout = controller->lout();
			for (Size i = 0; i < 36; ++i)
			{
				lout <<fwd.Jf()[i] << ",";
			}
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				if (param.joint_active_vec[i])
				{
					lout << param.begin_joint_pos_vec[i] << ",";
					lout << param.begin_axis_vel_vec[i] << ",";
					lout << param.begin_axis_acc_vec[i] << ",";
					lout << param.joint_pos_vec[i] << ",";
					lout << param.axis_vel_vec[i] << ",";
					lout << param.axis_acc_vec[i] << ",";
					lout << std::endl;
				}
			}

			// log 电流 //
            //auto &lout = controller->lout();
            //for (Size i = 0; i < 6; i++)
            //{
                //lout << controller->motionAtAbs(i).targetPos() << ",";
                //lout << controller->motionAtAbs(i).actualPos() << ",";
                //lout << controller->motionAtAbs(i).actualVel() << ",";
                //lout << controller->motionAtAbs(i).actualCur() << ",";
            //}
            //lout << std::endl;

            //return total_count - target.count;
            return total_count;
		}
	auto MoveTTT::collectNrt(PlanTarget &target)->void {}
	MoveTTT::MoveTTT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
            "<Command name=\"moveTTT\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
            "		<Param name=\"vel\" default=\"0.04\"/>"
            "		<Param name=\"acc\" default=\"0.1\"/>"
            "		<Param name=\"dec\" default=\"0.1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节混合插值梯形轨迹；速度前馈 //
	struct MoveJMParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		bool ab;
	};
	auto MoveJM::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveJMParam param;
			param.total_count_vec.resize(6, 1);
			param.axis_begin_pos_vec.resize(6, 0.0);

			//params.at("pos")
			for (auto &p : params)
			{
				if (p.first == "pos")
				{
					if (p.second == "current_pos")
					{
						target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					}
					else
					{
						auto pos = target.model->calculator().calculateExpression(p.second);
						if (pos.size() == 1)
						{
							param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
						}
						else if (pos.size() == param.axis_begin_pos_vec.size())
						{
							param.axis_pos_vec.assign(pos.begin(), pos.end());
						}
						else
						{
							throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						}

						for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
						{
							//超阈值保护//
							if (param.axis_pos_vec[i] > 1.0)
							{
								param.axis_pos_vec[i] = 1.0;
							}
							if (param.axis_pos_vec[i] < -1.0)
							{
								param.axis_pos_vec[i] = -1.0;
							}
							if (param.axis_pos_vec[i] >= 0)
							{
								param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].maxPos();
							}
							else
							{
								param.axis_pos_vec[i] = param.axis_pos_vec[i] * c->motionPool()[i].minPos();
							}					
						}
					}
				}
				else if (p.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
					}
					else if (v.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
						//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
					}
					else if (a.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
					}
					else if (d.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
				else if (p.first == "ab")
				{
					param.ab = std::stod(p.second);
				}
			}
			target.param = param;

			target.option |=
                //Plan::USE_VEL_OFFSET |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJM::executeRT(PlanTarget &target)->int
		{
			//获取驱动//
			auto controller = target.controller;
			auto &param = std::any_cast<MoveJMParam&>(target.param);
			static double begin_pos[6];
			static double pos[6];
			// 取得起始位置 //
			if (target.count == 1)
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
				}
			}
			// 设置驱动器的位置 //
			if (param.ab)
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					double p, v, a;
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
						, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
					controller->motionAtAbs(i).setTargetPos(p);
					//速度前馈//
					controller->motionAtAbs(i).setOffsetVel(v * 1000);
					target.model->motionPool().at(i).setMp(p);
				}
			}
			else
			{
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					double p, v, a;
					aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
						, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
					controller->motionAtAbs(i).setTargetPos(p);
					//速度前馈//
					controller->motionAtAbs(i).setOffsetVel(v * 1000);
					target.model->motionPool().at(i).setMp(p);
				}
			}
				
			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
		}
	auto MoveJM::collectNrt(PlanTarget &target)->void {}
	MoveJM::MoveJM(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"		<Param name=\"ab\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 关节插值运动轨迹--输入末端pq姿态，各个关节的速度、加速度；各关节按照梯形速度轨迹执行；速度前馈 //
	struct MoveJIParam
	{
		std::vector<double> pq;
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto MoveJI::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
		{
			auto c = target.controller;
			MoveJIParam param;
			param.pq.resize(7, 0.0);
			param.total_count_vec.resize(6, 1);
			param.axis_begin_pos_vec.resize(6, 0.0);
			param.axis_pos_vec.resize(6, 0.0);

			//params.at("pq")
			for (auto &p : params)
			{
				if (p.first == "pq")
				{
					if (p.second == "current_pos")
					{
						target.option |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					}
					else
					{
						auto pqarray = target.model->calculator().calculateExpression(p.second);
						param.pq.assign(pqarray.begin(), pqarray.end());
					}
				}
				else if (p.first == "vel")
				{
					auto v = target.model->calculator().calculateExpression(p.second);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(param.axis_pos_vec.size(), v.toDouble());
					}
					else if (v.size() == param.axis_pos_vec.size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
						//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = target.model->calculator().calculateExpression(p.second);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(param.axis_pos_vec.size(), a.toDouble());
					}
					else if (a.size() == param.axis_pos_vec.size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = target.model->calculator().calculateExpression(p.second);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(param.axis_pos_vec.size(), d.toDouble());
					}
					else if (d.size() == param.axis_pos_vec.size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}	
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}
			target.param = param;

			target.option |=
				Plan::USE_VEL_OFFSET |
#ifdef WIN32
				Plan::NOT_CHECK_POS_MIN |
				Plan::NOT_CHECK_POS_MAX |
				Plan::NOT_CHECK_POS_CONTINUOUS |
				Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
				Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
				Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
				Plan::NOT_CHECK_VEL_MIN |
				Plan::NOT_CHECK_VEL_MAX |
				Plan::NOT_CHECK_VEL_CONTINUOUS |
				Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
				Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

		}
	auto MoveJI::executeRT(PlanTarget &target)->int
		{
			//获取驱动//
			auto controller = target.controller;
			auto &param = std::any_cast<MoveJIParam&>(target.param);
			static double begin_pos[6];
			static double pos[6];
			// 取得起始位置 //
			if (target.count == 1)
			{
				target.model->generalMotionPool().at(0).setMpq(param.pq.data());	//generalMotionPool()指模型末端，at(0)表示第1个末端，对于6足就有6个末端，对于机器人只有1个末端
				if (!target.model->solverPool().at(0).kinPos())return -1;
				for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
				{
					param.axis_begin_pos_vec[i] = controller->motionPool().at(i).targetPos();
					param.axis_pos_vec[i] = target.model->motionPool().at(i).mp();		//motionPool()指模型驱动器，at(0)表示第1个驱动器
				}
			}
			// 设置驱动器的位置 //
			for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller->motionAtAbs(i).setTargetPos(p);
				//速度前馈//
				controller->motionAtAbs(i).setOffsetVel(v*1000);
				target.model->motionPool().at(i).setMp(p);
			}		
			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 100 == 0)
			{
				for (Size i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
					cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
					cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();
			for (Size i = 0; i < 6; i++)
			{
				lout << controller->motionAtAbs(i).targetPos() << ",";
				lout << controller->motionAtAbs(i).actualPos() << ",";
				lout << controller->motionAtAbs(i).actualVel() << ",";
				lout << controller->motionAtAbs(i).actualCur() << ",";
			}
			lout << std::endl;

			return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
		}
	auto MoveJI::collectNrt(PlanTarget &target)->void {}
	MoveJI::MoveJI(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJI\">"
			"	<GroupParam>"
			"		<Param name=\"pq\" default=\"current_pos\"/>"
            "		<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.05,0.05,0.05}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(aris::robot::createPlanRootRokaeXB4());

		plan_root->planPool().add<aris::plan::MoveL>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();;
		plan_root->planPool().add<kaanh::MoveX>();
		plan_root->planPool().add<kaanh::MoveJS>();
		plan_root->planPool().add<kaanh::MoveJSN>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::MoveTTT>();
		plan_root->planPool().add<kaanh::MoveJM>();
		plan_root->planPool().add<kaanh::MoveJI>();

		return plan_root;
	}

}
