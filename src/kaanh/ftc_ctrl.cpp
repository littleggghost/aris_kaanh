#pragma once
#include "kaanh.h"
#include<math.h>
#include"aris.hpp"
using namespace aris::dynamic;
using namespace std;
//using namespace cpt_ftc;
//struct FilterParam
//{
//	const double PI = 3.14159265;
//	double Wc;
//	double a[3];
//	double b[3];
//	double x_pre;
//	double x_last;
//	double y_pre;
//	double y_last;
//};
//
//struct LowPass::Imp :public FilterParam {};



namespace cpt_ftc
{
    void cpt_ftc::LowPass::get_filter_data(int n_order, int fc, double Ts, double x_in, double &x_out)
	{
		Wc = 2 * PI*fc;
		if (n_order == 1)
		{
			a[1] = (Wc*Ts - 2) / (Wc*Ts + 2);
			b[0] = b[1] = Wc * Ts / (Wc*Ts + 2);
			x_out = b[0] * x_in + b[1] * x_last - a[1] * y_last;
			//x_pre = x_last;
			x_last = x_in;
			y_last = x_out;
		}
		if (n_order == 2)
		{
			den = Wc * Wc * Ts*Ts + 2 * sqrt(2)*Wc*Ts + 4;
			a[1] = (2 * Wc * Wc * Ts*Ts - 8) / den;
			a[2] = (Wc * Wc * Ts*Ts - 2 * sqrt(2)*Wc*Ts + 4) / den;
			b[0] = b[2] = Wc * Wc * Ts*Ts / den;
			b[1] = 2 * b[0];
			x_out = b[0] * x_in + b[1] * x_last + b[2] * x_pre - (a[1] * y_last + a[2] * y_pre);
			x_pre = x_last;
			x_last = x_in;
			y_pre = y_last;
			y_last = x_out;
		}

		//struct Imp;
		//aris::core::ImpPtr<Imp> imp_;

	}
	//LowPass low_pass;
	void cpt_ftc::Admit::admit_init(double ft[6],double g[6])
	{
		fill_n(cor_vel, 6, 0);
		fill_n(cor_pos_now, 6, 0);
		fill_n(cor_pos, 6, 0);
		fill_n(pm_now, 16, 0);
		pm_now[0] = pm_now[5] = pm_now[10] = pm_now[15] = 1.0;		
		for (int i = 0; i < 6; i++)
		{
			G[i] = g[i];
			ft_offset[i] = ft[i] - G[i];
		}
	}
	auto cpt_ftc::Admit::get_cor_pos(double ft[6], double pm_fce2target[16], double dt)->std::array<double,6>
	{
		double ft_ext_sensor[6];
		for (int i = 0; i < 6; i++)
		{
			ft_ext_sensor[i] = ft[i] - G[i] - ft_offset[i];
		}
		double ft_ext_target[6];//external force in target coordinate
		s_fs2fs(pm_fce2target, ft_ext_sensor, ft_ext_target);
		double ft_err[6];
		for (int i = 0; i < 6; i++)
		{
			ft_err[i] = ft_set[i] - ft_ext_target[i];
			if (std::abs(ft_err[i]) < dead_zone[i])
			{
				ft_err[i] = 0;
			}
			else
			{
				ft_err[i] = ft_err[i] < 0 ? (ft_err[i] + dead_zone[i]) : (ft_err[i] - dead_zone[i]);
			}
		}


		for (int i = 0; i < 3; i++)
		{
			cor_vel[i] = ft_err[i] / B[i];
			cor_pos_now[i] = std::min(std::max(cor_vel[i], -vel_limit[i]), vel_limit[i])*dt;
			cor_pos[i] += cor_pos_now[i];
		}
		for (int i = 3; i < 6; i++)
		{
			cor_vel[i] = ft_err[i] / B[i];
			cor_vel[i] = std::min(std::max(cor_vel[i], -vel_limit[i]), vel_limit[i]);
		}
		double Norm;
		double theta;
		double q[4];
		Norm = sqrt(pow(cor_vel[3], 2) + pow(cor_vel[4], 2) + pow(cor_vel[5], 2));
		if (std::abs(Norm) > 1e-10)
		{
			theta = Norm * dt;
			cor_vel[3] = cor_vel[3] / Norm;
			cor_vel[4] = cor_vel[4] / Norm;
			cor_vel[5] = cor_vel[5] / Norm;
			q[0] = cos(theta / 2.0);
			q[1] = cor_vel[3] * sin(theta / 2.0);
			q[2] = cor_vel[4] * sin(theta / 2.0);
			q[3] = cor_vel[5] * sin(theta / 2.0);
			double pq[7]{ cor_pos_now[0], cor_pos_now[1], cor_pos_now[2], q[0], q[1], q[2], q[3] };
			double pm[16];
			s_pq2pm(pq, pm);
			double pm_next[16];
			s_pm_dot_pm(pm_now, pm, pm_next);
			double pe_next[6];
			s_pm2pe(pm_next, pe_next, "321");
			s_vc(16, pm_next, pm_now);
			//for (int i = 0; i < 3; i++)
			//{
			//	cor_pos[i] = pe_next[i];
			//}
			for (int i = 3; i < 6; i++)
			{
				while (pe_next[i] > PI)pe_next[i] -= 2 * PI;
				while (pe_next[i] < -PI)pe_next[i] += 2 * PI;
				/*cor_pos_now[i] = pe_next[i];
				cor_pos[i] += cor_pos_now[i];*/
				cor_pos[i] = pe_next[i];
			}
		}
		for (int i = 0; i < 6; i++)
		{
			cor_pos[i] = std::min(std::max(cor_pos[i], -cor_pos_limit[i]), cor_pos_limit[i]);

		}
		//}
		return std::array<double, 6>{cor_pos[0], cor_pos[1],cor_pos[2],cor_pos[3],cor_pos[4],cor_pos[5]};
	}

	// admit init//
	

}

