#ifndef KAANH_H_
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

    auto createController()->std::unique_ptr<aris::control::Controller>;
    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;

    class FSSignal : public aris::plan::Plan
    {
    public:
        auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
        auto virtual executeRT(aris::plan::PlanTarget &target)->int;
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

        explicit FSSignal(const std::string &name = "FSSignal_plan");
        ARIS_REGISTER_TYPE(FSSignal);
    };

    class ATIFS : public aris::plan::Plan
    {
    public:
        auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
        auto virtual executeRT(aris::plan::PlanTarget &target)->int;
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

        explicit ATIFS(const std::string &name = "ATIFS_plan");
        ARIS_REGISTER_TYPE(ATIFS);
    };
}

#endif
