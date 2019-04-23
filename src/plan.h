#ifndef ROKAE_PLAN_H_
#define ROKAE_PLAN_H_

#include <aris.h>

using namespace aris::plan;

struct MoveJSParam
{
	int total_time;
	int left_time;
	double step_size;//
};
class MoveJS : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveJSParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.step_size = std::stod(params.at("step_size"));//
		p.left_time = 0;
		target.param = p;
		target.option =
			//����λ����Բ��ý�model�Ĺ켣��ֵ��controller���棬ϵͳֱ�ӵ���model�еķ�������������
			//������������ô��Ҫ��forѭ����model�еķ����������ֵ��controller����
			//aris::plan::Plan::USE_TARGET_POS |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;

		
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveJSParam&>(target.param);

		static double beginpos[6] = { 0, 0, 0, 0, 0, 0 };
		if (target.count == 1)
		{
			target.master->mout() << "mot1:" << controller->motionPool()[5].actualPos() << std::endl;
			//target.master->mout() << "mot2:" << controller->motionPool()[1].actualPos() << std::endl;

			beginpos[5] = controller->motionPool()[5].actualPos();
			//beginpos[1] = controller->motionPool()[1].actualPos();

			//double pq[7]{ 0,0,0,0,0,0,1 };

			
		}

		static double beginpq[7];
		if (target.count == 1)
		{
			//��ȡ��ǰģ�͵�λ��
			target.model->generalMotionPool()[0].getMpq(beginpq);
		}


		//����һ����������Ԫ����ǰ��������λ��
		//double pq[7];
		//����ȡ�Ļ�����λ�ø�ֵ������
		//std::copy_n(beginpq, 7, pq);
		//�Ա����ĵ�һ�����������˶��滮
		//pq[0] = beginpq[0] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
		//��������ֵ��ֵ��model��ģ�͵�ĩ��λ��
		//target.model->generalMotionPool()[0].setMpq(pq);
		//���˶�ѧ������Ӵ�����������solverpool��kinPos��λ�÷��⣬kinVel���ٶȷ���
		//target.model->solverPool()[0].kinPos();

		

		


		controller->motionPool()[5].setTargetPos(beginpos[5] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI))); //0.01
		//controller->motionPool()[0].setTargetPos(beginpos[0] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI))); //0.01
		//controller->motionPool()[1].setTargetPos(beginpos[1] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI)));

		return p.total_time - target.count;
	}
	auto virtual collectNrt(PlanTarget &target)->void {}
	explicit MoveJS(const std::string &name = "myplan") :Plan(name)
	{
		command().loadXmlStr(
			"<myplan>"
			"	<group type=\"GroupParam\">"
			"	    <total_time type=\"Param\" default=\"5000\"/>" //Ĭ��5000
            "       <step_size type=\"Param\" default=\"0.1\"/>"
			"   </group>"
			"</myplan>");
	}
};

#endif