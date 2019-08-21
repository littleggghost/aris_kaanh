#include <algorithm>
#include "kaanh.h"
#include "xyz.h"
#include <array>
#include <stdlib.h>


using namespace aris::dynamic;
using namespace aris::plan;


namespace xyz
{
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

		std::fill(target.mot_options.begin(), target.mot_options.end(),
			Plan::USE_TARGET_POS);

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

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
				//target.controller->motionPool().at(i).setTargetPos(step_pjs[i]);
			}
		}
		else if ((time / 2 < target.count) && (target.count <= totaltime - time / 2))
		{
			// 获取当前起始点位置 //
			if (target.count == time / 2 + 1)
			{
				for (Size i = 0; i < param.joint_active_vec.size(); ++i)
				{
					begin_pjs[i] = target.model->motionPool()[i].mp();
					step_pjs[i] = target.model->motionPool()[i].mp();
				}
			}
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				step_pjs[i] = begin_pjs[i] - 2 * param.j[i] * (1 - std::cos(2 * PI*(target.count - time / 2) / time)) / 2;
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

		if (target.model->solverPool().at(1).kinPos())return -1;

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
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualToq() << "  ";
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
			lout << controller->motionAtAbs(i).actualToq() << ",";
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


	// 多关节混合插值梯形轨迹 //
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

		std::vector<std::pair<std::string, std::any>> ret;
		target.ret = ret;

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
				
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).actualPos() << ",";
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


    auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<aris::plan::Enable>();
        plan_root->planPool().add<aris::plan::Disable>();
        plan_root->planPool().add<aris::plan::Home>();
        plan_root->planPool().add<aris::plan::Mode>();
        plan_root->planPool().add<aris::plan::Show>();
        plan_root->planPool().add<aris::plan::Sleep>();
        plan_root->planPool().add<aris::plan::Recover>();
        auto &rs = plan_root->planPool().add<aris::plan::Reset>();
		
		//for qifan robot//
        rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");
		
		//for rokae robot//
        //rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");

        plan_root->planPool().add<aris::plan::MoveAbsJ>();
        plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<kaanh::Get>();
		plan_root->planPool().add<kaanh::JogC>();
		plan_root->planPool().add<kaanh::JogJ>();
		plan_root->planPool().add<kaanh::JogJ1>();
		plan_root->planPool().add<kaanh::JogJ2>();
		plan_root->planPool().add<kaanh::JogJ3>();
		plan_root->planPool().add<kaanh::JogJ4>();
		plan_root->planPool().add<kaanh::JogJ5>();
		plan_root->planPool().add<kaanh::JogJ6>();
		plan_root->planPool().add<kaanh::JX>();
		plan_root->planPool().add<kaanh::JY>();
		plan_root->planPool().add<kaanh::JZ>();
		plan_root->planPool().add<kaanh::JRX>();
		plan_root->planPool().add<kaanh::JRY>();
		plan_root->planPool().add<kaanh::JRZ>();

		//添加自定义指令//
		plan_root->planPool().add<xyz::MoveJS>();
		plan_root->planPool().add<xyz::MoveJM>();

		return plan_root;
	}

}
