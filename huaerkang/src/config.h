#ifndef CONFIG_H_
#define CONFIG_H_

#include <memory>
#include <aris.hpp>
#include "kaanh/kaanhconfig.h"
#include "kaanh.h"

//global time speed array//
//extern double timespeed[101];

namespace config
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
    constexpr int interval = 2;//指令周期，单位ms

    //获取当前位置/速度指令
    class GVel : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        explicit GVel(const std::string &name = "GVel");
        ARIS_REGISTER_TYPE(GVel);
    };

    //单轴运动指令
	class MoveAbs :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveAbs();
        explicit MoveAbs(const std::string &name = "moveabs");
        ARIS_REGISTER_TYPE(MoveAbs);
	};

	class MoveLine :public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveLine();
        explicit MoveLine(const std::string &name = "moveline");
        ARIS_REGISTER_TYPE(config::MoveLine);
	};

    class MoveJoint : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

        virtual ~MoveJoint();
        explicit MoveJoint(const std::string &name = "movejoint");
        ARIS_REGISTER_TYPE(config::MoveJoint);

	};

    class MoveStep : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MoveStep();
        explicit MoveStep(const std::string &name = "movestep");
        ARIS_REGISTER_TYPE(config::MoveStep);

    };

    //暂停指令
    class PS : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        explicit PS(const std::string &name = "PS");
        ARIS_REGISTER_TYPE(PS);
    };

    //急停指令
    class EStop : public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        explicit EStop(const std::string &name = "EStop");
        ARIS_REGISTER_TYPE(EStop);
    };

    auto createController()->std::unique_ptr<aris::control::Controller>;
    auto createModel(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
