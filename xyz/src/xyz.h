#ifndef XYZ_H_
#define XYZ_H_

#include <memory>
#include <aris.hpp>


namespace xyz
{
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;

	class MoveJS : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJS(const std::string &name = "MoveJS_plan");
		ARIS_REGISTER_TYPE(MoveJS);
	};

	class MoveJM : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJM(const std::string &name = "MoveJM_plan");
		ARIS_REGISTER_TYPE(MoveJM);
	};

}

#endif
