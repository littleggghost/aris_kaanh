﻿#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_

#include <memory>
#include <aris.hpp>
#include <atomic>

//其他参数和函数声明 
using Size = std::size_t;
//constexpr double PI = 3.141592653589793;
long long timetmp = 0;

class MoveJS : public aris::plan::Plan
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJS(const std::string &name = "MoveJS_plan");
    ARIS_REGISTER_TYPE(MoveJS);
};

class Move : public aris::plan::Plan
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit Move(const std::string &name = "Move_plan");
    ARIS_REGISTER_TYPE(Move);
};

class MoveAbs :public aris::plan::Plan
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;

    virtual ~MoveAbs();
    explicit MoveAbs(const std::string &name = "mvaj");
    ARIS_REGISTER_TYPE(MoveAbs);
};

#endif
