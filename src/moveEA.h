#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris.hpp>
#include <atomic>


//电缸力检测参数声明
constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//

//其他参数和函数声明 
using Size = std::size_t;
constexpr double PI = 3.141592653589793;

class MoveJS : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit MoveJS(const std::string &name = "MoveJS_plan");
	ARIS_REGISTER_TYPE(MoveJS);
};


class MoveEAP : public aris::plan::Plan
{
public:
    auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
    auto virtual executeRT(aris::plan::PlanTarget &target)->int;
    auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

    explicit MoveEAP(const std::string &name = "MoveEAP_plan");
    ARIS_REGISTER_TYPE(MoveEAP);
};


#endif
