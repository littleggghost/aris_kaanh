﻿#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>


// \brief 机器人命名空间
// \ingroup aris
//


namespace kaanh
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;

	auto registerPlan()->void;

	class MoveJRT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJRT(const std::string &name = "MoveJRT_plan");
		ARIS_REGISTER_TYPE("MoveJRT");
	};

}

#endif
