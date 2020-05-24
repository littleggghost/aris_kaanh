#include "kaanh/kinematic.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>


using namespace aris::plan;
using namespace aris::dynamic;

/*
����4��궨��������ϵ������ԭ��TCP����������4����ͬ����TCP��һ���̶���Ӵ������ݶ�Ӧ�ķ�������ϵ��λ�˼���TCP
����5��궨��������ϵ������ԭ��TCP�ͺ�TCF��Z�᷽��
����6��궨��������ϵ������ԭ��TCP�ͺ�TCF��Z�ᡢY�᷽��
*/

//�������
auto crossVector(double *a, double *s, double *n)->int
{
	//n=a��s
	n[0] = a[1] * s[2] - s[1] * a[2];
	n[1] = s[0] * a[2] - a[0] * s[2];
	n[2] = a[0] * s[1] - s[0] * a[1];
	return 0;
}

//��λ�ú�ŷ������ɵ�λ�˵ĵ�λ��mm�͡�ת��Ϊm��rad
auto mmdeg2mrad(std::string datastr, double *data, size_t data_size)->int
{
	std::string posestr = datastr.substr(datastr.find("{") + 1, datastr.find("}") - datastr.find("{") - 1);
	std::string::size_type pos1 = 0;
	std::string::size_type pos2 = posestr.find(",");
	std::vector<std::string> tempvec;
	while (pos2 != std::string::npos)
	{
		tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
		pos1 = pos2 + 1;
		pos2 = posestr.find(",", pos1);
	}
	tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
	if (tempvec.size() == data_size)
	{
		for (size_t i = 0; i < data_size; i++)
		{
			size_t j = i % 6;
			if (j<3)
			{ 
				data[i] = std::stod(tempvec.at(i))/1000; 
			}
			else
			{
				data[i] = std::stod(tempvec.at(i)) * PI / 180;
			}
		}
		return 0;
	}
	else
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong��");		//"�����ʾ�̵�λ����������"
		return -1;
	}
}

//��ȡʾ�̵�λ������
auto get_teachpt_data(std::string datastr, double *data, size_t data_size)->int
{
	std::string posestr = datastr.substr(datastr.find("{") + 1, datastr.find("}") - datastr.find("{") - 1);
	std::string::size_type pos1 = 0;
	std::string::size_type pos2 = posestr.find(",");
	std::vector<std::string> tempvec;
	while (pos2 != std::string::npos)
	{
		tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
		pos1 = pos2 + 1;
		pos2 = posestr.find(",", pos1);
	}
	tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
	if (tempvec.size() == data_size)
	{
		for (size_t i = 0; i < data_size; i++)
		{
			data[i] = std::stod(tempvec.at(i));
		}
		return 0;
	}
	else
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong��");		//"�����ʾ�̵�λ����������"
		return -1;
	}
}


//����3��궨����������ϵ��3�㲻����
//ʾ�����̣���ʾ�̹�������ϵԭ�㣬Ȼ��ʾ�̹�������ϵx���ϵ�һ�㣬���ʾ�̹���ƽ���������һ����
struct CalibW3PParam
{
	std::vector<double> pe_3pt;
	std::vector<double> wobj_pe;
	std::string wobj_name;
	std::string calib_info;

};
auto CalibW3P::prepareNrt()->void
{
	//������ʼ��
	CalibW3PParam param;
	param.pe_3pt.clear();
	param.pe_3pt.resize(18,0.0);
	param.wobj_pe.resize(6, 0.0);
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			auto temp = this->matrixParam(p.first);
			if (temp.size() == param.pe_3pt.size())
			{
				param.pe_3pt.assign(temp.begin(), temp.end());
			}
			else
			{
				param.calib_info = std::string(" Input data error, the calculation process was aborted!").c_str();
			}
		}
		else if (p.first == "wobj")
		{
			param.wobj_name = std::string(p.second);
		}
	}
	this->param() = param;

	double zero[3] = { param.pe_3pt[0], param.pe_3pt[1], param.pe_3pt[2] };//��������ϵԭ��
	double x_raw[3] = { param.pe_3pt[6] - param.pe_3pt[0], param.pe_3pt[7] - param.pe_3pt[1], param.pe_3pt[8] - param.pe_3pt[2] };//��ù�������ϵx������
	double xy_raw[3] = { param.pe_3pt[12] - param.pe_3pt[0], param.pe_3pt[13] - param.pe_3pt[1], param.pe_3pt[14] - param.pe_3pt[2] };//��ù�������ϵxy��Ļ�ϲ���x����һ��
	double y_raw[3], z_raw[3];

	auto norm_x = aris::dynamic::s_norm(3, x_raw);
	if(std::abs(norm_x) < 1e-9)
	{
		param.calib_info = std::string("The teaching points are in line, The calculation process was aborted!").c_str();
		throw std::runtime_error("The teaching points are in line");
	}
	double x[3] = { x_raw[0] / norm_x, x_raw[1] / norm_x, x_raw[2] / norm_x };//��������ϵx�᷽������

	crossVector(x_raw, xy_raw, z_raw);
	auto norm_z = aris::dynamic::s_norm(3, z_raw);
	if (std::abs(norm_z) < 1e-9)
	{
		param.calib_info = std::string("The teaching points are in line, The calculation process was aborted!").c_str();
		throw std::runtime_error("The teaching points are in line");
	}
	double z[3] = { z_raw[0] / norm_z, z_raw[1] / norm_z,  z_raw[2] / norm_z };//��������ϵz�᷽������

	crossVector(z_raw, x_raw, y_raw);
	auto norm_y = aris::dynamic::s_norm(3, y_raw);
	if (std::abs(norm_y) < 1e-9)
	{
		param.calib_info = std::string("The teaching points are in line, The calculation process was aborted!").c_str();
		throw std::runtime_error("The teaching points are in line");
	}
	double y[3] = { y_raw[0] / norm_y, y_raw[1] / norm_y,  y_raw[2] / norm_y };//��������ϵy�᷽������

	double wobj[16]{ x[0], y[0], z[0], zero[0], x[1], y[1], z[1], zero[1], x[2], y[2], z[2], zero[2], 0, 0, 0, 1 };

	//���궨���ת��Ϊŷ������ʽ
	aris::dynamic::s_pm2pe(wobj, param.wobj_pe.data());

	try
	{
		model()->partPool().findByName("ground")->markerPool().findByName(param.wobj_name)->setPrtPm(wobj);
	}
	catch (std::exception)
	{
		param.calib_info = std::string("cann't find \"" + param.wobj_name + "\" node in markerPool.");
		throw std::runtime_error("cann't find \"" + param.wobj_name + "\" node in markerPool.");
	}

	/*
	if (model()->partPool().findByName("ground")->markerPool().findByName(param.wobj_name) != model()->partPool().findByName("ground")->markerPool().end())
	{
		dynamic_cast<aris::dynamic::Marker*>(&*model()->partPool().findByName("ground")->markerPool().findByName(param.wobj_name))->setPrtPm(wobj);
	}
	else
	{
		model()->partPool().findByName("ground")->markerPool().add<aris::dynamic::Marker>(param.wobj_name, wobj);
	}
	*/

	auto xmlpath = std::filesystem::absolute(".");
	const std::string xmlfile = "kaanh.xml";
	xmlpath = xmlpath / xmlfile;
	controlServer()->saveXmlFile(xmlpath.string().c_str());

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("wobj_pe", param.wobj_pe));
	ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
}
CalibW3P::CalibW3P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"CalibW3P\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"wobj\" default=\"wobj1\"/>"
		"   </GroupParam>"
		"</Command>");
}


//4�㷨�궨��������ϵ
struct CalibT4PParam
{
	double pe_4pt[24];
	std::vector<double> tool_pe;
	std::string calib_info;
	std::string tool_name;
	
};
auto CalibT4P::prepareNrt()->void
{
	//������ʼ��
	CalibT4PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr=std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_4pt, 24);
			if (ret1 != 0) return;
		}
		else if (p.first == "tool")
		{
			param.tool_name = std::string(p.second);
		}
	}
	this->param() = param;
	double pm_4pt[64];
	double tcp[3];				//�����õ�tcp
	double tcp_error = 0;		//����tcp�����
	//double tcf[9];			//�����õ�tcf
	for (int i = 0; i < 4; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_4pt[6 * i + j];
		}
		s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_4pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret2 = cal_TCP(pm_4pt, tcp, tcp_error);
	if (ret2 == 0)
	{
		//���궨���ת��Ϊŷ������ʽ
		const double pose[6] = { tcp[0], tcp[1], tcp[2], 0, 0, 0 };
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe.push_back(pose[i]);
		}
		const std::string calib_info = "The calculation is done! The calibration error of TCP is:" + std::to_string(tcp_error * 1000) + "mm";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		param.calib_info = std::string("The calculation process was aborted!").c_str();
		throw std::runtime_error("The calculation process was aborted!");		//�޷�����궨�������ȡ��ʾ�̵��쳣��������ִ�б궨����
	}

	//����궨���
	{
		//��ȡ��������ϵ����ڷ�������ϵ��λ��
		double tool_pm_f[16];
		s_pe2pm(param.tool_pe.data(), tool_pm_f, "321");
		//��ȡ��������ϵ����ڵ�������ϵ��λ��
		double tool0_pm_g[16];
		try
		{
			auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName("tool0")->prtPm();
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					tool0_pm_g[4 * i + j] = mat1[i][j];
				}
			}
		}
		catch (std::exception)
		{
			std::string calib_info = "cann't find \"tool0\" node in partPool.";
			throw std::runtime_error("cann't find \"tool0\" node in partPool.");
		}

		//���㹤������ϵ����ڵ�������ϵ��λ��
		double tool_pm_g[16];
		double tool_pe_g[6];
		s_mm(4, 4, 4, tool0_pm_g, tool_pm_f, tool_pm_g);
		s_pm2pe(tool_pm_g, tool_pe_g, "313");
		try
		{
			model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->setPrtPe(tool_pe_g);
		}
		catch (std::exception)
		{
			std::string calib_info = "cann't find \"" + param.tool_name + "\" node in partPool.";
			throw std::runtime_error("cann't find \"" + param.tool_name + "\" node in partPool.");
		}

		const std::string calib_info = "The configuration node of " + param.tool_name + "'s pose is created or updated.";
		param.calib_info = calib_info.c_str();
	}

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibT4P::CalibT4P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"CalibT4P\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"tool\" default=\"tool1\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT4P::cal_TCP(double transmatric[64], double tcp[3], double &tcp_error)->int
{
	//����R(i)-R(i+1),P(i+1)-P(i)
	double R[36], P[12];
	tm2RP_4Pt(transmatric, R, P);
	/*for (int i = 0; i < 12; i++)
	{
		std::cout << P[i] << ",";
	}*/
	double deltaR[27], deltaP[9];
	deltaRP_4Pt(R, P, deltaR, deltaP);

	//����tcp��tcp_error
	double U[27] = { 0 };
	double tau[9] = { 0 };
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	//�ж��Ƿ�������
	if (rank < 3)
	{
		//throw std::runtime_error("ʾ�̵������쳣�������»�ȡʾ�̵㣡");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//����tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);
	/*for (int i = 0; i < 9; i++)
	{
		std::cout << new_deltaP[i] << ","<<std::endl;
	}*/
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
		//std::cout << error_vec[i] << "," << std::endl;
	}
	double prod_error[3] = { 0.0, 0.0, 0.0 };
	double sum_prod_error = 0.0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);
	//std::cout << "���" << tcp_error;
	return 0;
}
auto CalibT4P::tm2RP_4Pt(double tm[64], double *R, double *P)->int
{
	//��n_pt*16����任��������ȡn_pt*9��ת�����n_pt*3ƽ������
	int num = 64;
	int n_pt = 4;
	double temp[4][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT4P::deltaRP_4Pt(double R[36], double P[12], double * deltaR, double * deltaP)->int
{
	//����R(i)-R(i+1),P(i+1)-P(i),i=ʾ�̵�����n_pt��1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//5�㷨�궨��������ϵ
struct CalibT5PParam
{
	double pe_5pt[30];
	std::vector<double> tool_pe;
	std::string calib_info;
	std::string tool_name;
};
auto CalibT5P::prepareNrt()->void
{
	//������ʼ��
	CalibT5PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr = std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_5pt, 30);
			if (ret1 != 0) return;
		}
		else if (p.first == "tool")
		{
			param.tool_name = std::string(p.second);
		}
	}
	this->param() = param;
	double pm_5pt[80];
	double tcp[3];		//�����õ�tcp
	double tcp_error;		//����tcp�����
	double tcf[9];		//�����õ�tcf
	for (int i = 0; i < 5; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_5pt[6 * i + j];
		}
		s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_5pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret = cal_TCP_Z(pm_5pt, tcp, tcp_error, tcf);
	if (ret == 0)
	{
		//���궨���ת��Ϊŷ������ʽ
		double re321[3];
		s_rm2re(tcf, re321, "321");
		const double pose[6] = { tcp[0], tcp[1], tcp[2], re321[0], re321[1], re321[2] };
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe.push_back(pose[i]);
		}
		//param.calib_info = "��������ϵ5��궨��ɣ��������ĵ㣨TCP�����������ǣ�" + std::to_string(tcp_error * 1000) + "mm";
		const std::string calib_info = "The calculation is done! The calibration error of TCP is:" + std::to_string(tcp_error * 1000) + "mm";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("The calculation process was aborted!");		//�޷�����궨�������ȡ��ʾ�̵��쳣��������ִ�б궨���̡�
		param.calib_info = std::string("The calculation process was aborted!").c_str();
		//return;
	}
	
	//����궨���
	{
		//��ȡ��������ϵ����ڷ�������ϵ��λ��
		double tool_pm_f[16];
		s_pe2pm(param.tool_pe.data(), tool_pm_f, "321");
		//��ȡ��������ϵ����ڵ�������ϵ��λ��
		double tool0_pm_g[16];
		try
		{
			auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName("tool0")->prtPm();
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					tool0_pm_g[4 * i + j] = mat1[i][j];
				}
			}
		}
		catch (std::exception)
		{
			std::string calib_info = "cann't find \"tool0\" node in partPool.";
			throw std::runtime_error("cann't find \"tool0\" node in partPool.");
		}

		//���㹤������ϵ����ڵ�������ϵ��λ��
		double tool_pm_g[16];
		double tool_pe_g[6];
		s_mm(4, 4, 4, tool0_pm_g, tool_pm_f, tool_pm_g);
		s_pm2pe(tool_pm_g, tool_pe_g, "313");
		try
		{
			model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->setPrtPe(tool_pe_g);
		}
		catch (std::exception)
		{
			std::string calib_info = "cann't find \"" + param.tool_name + "\" node in partPool.";
			throw std::runtime_error("cann't find \"" + param.tool_name + "\" node in partPool.");
		}

		const std::string calib_info = "The configuration node of " + param.tool_name + "'s pose is created or updated.";
		param.calib_info = calib_info.c_str();
	}

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	this->ret()= out_param;
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibT5P::CalibT5P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"CalibT5P\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}\"/>"
		"		<Param name=\"tool\" default=\"tool1\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT5P::cal_TCP_Z(double transmatric[80], double tcp[3], double &tcp_error, double tcf[9])->int
{
	//����R(i)-R(i+1),P(i+1)-P(i)
	double R[45], P[15];
	tm2RP_5Pt(transmatric, R, P);
	double tcp_R[36], tcp_P[12], tcf_R[18], tcf_P[6];
	//��ȡ����tcp������
	for (int i = 0; i < 36; i++)
	{
		tcp_R[i] = R[i];
	}
	for (int i = 0; i < 12; i++)
	{
		tcp_P[i] = P[i];
	}
	//��ȡ����tcf������
	for (int i = 0; i < 18; i++)
	{
		tcf_R[i] = R[45 - 18 + i];
	}
	for (int i = 0; i < 6; i++)
	{
		tcf_P[i] = P[15 - 6 + i];
	}

	//����tcp
	double deltaR[27], deltaP[9];
	deltaRP_5Pt(tcp_R, tcp_P, deltaR, deltaP);
	double U[27], tau[9];
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	if (rank < 3)
	{
		//throw std::runtime_error("ʾ�̵������쳣�������»�ȡʾ�̵㣡");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//����tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
	}
	double prod_error[3] = { 0 };
	double sum_prod_error = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);

	//����tcf
	/*double tcf_deltaR[9], tcf_deltaP[3];
	deltaRP_5Pt(tcf_R, tcf_P, tcf_deltaR, tcf_deltaP);
	double _tcf_deltaR[9], temp_vec[3], oz_vec[3];
	for (int i = 0; i < 9; i++)
	{
		_tcf_deltaR[i] = -tcf_deltaR[i];
	}*/
	double tcf_deltaRz[9], tcf_deltaPz[3];
	double temp_vec[3], oz_vec[3];
	for (int i = 0; i < 9; i++)
	{
		tcf_deltaRz[i] = tcf_R[9 + i] - tcf_R[i];
	}
	for (int i = 0; i < 3; i++)
	{
		tcf_deltaPz[i] = tcf_P[i + 3] - tcf_P[i];
	}
	s_mm(3, 1, 3, tcf_deltaRz, tcp, temp_vec);
	for (int i = 0; i < 3; i++)
	{
		oz_vec[i] = temp_vec[i] + tcf_deltaPz[i];
	}
	double prod_oz = 0;
	double len_oz = 0;
	for (int i = 0; i < 3; i++)
	{
		prod_oz = prod_oz + oz_vec[i] * oz_vec[i];
	}
	len_oz = std::sqrt(prod_oz);
	double tcf_R0[9];
	for (int i = 0; i < 9; i++)
	{
		tcf_R0[i] = tcf_R[i];
	}
	double U_tcf[9], tau_tcf[3];
	aris::Size p_tcf[3];
	aris::Size rank_tcf;
	s_householder_utp(3, 3, tcf_R0, U_tcf, tau_tcf, p_tcf, rank_tcf, 1e-10);
	double tau2_tcf[3], pinv_tcf[9];
	s_householder_utp2pinv(3, 3, rank_tcf, U_tcf, tau_tcf, p_tcf, pinv_tcf, tau2_tcf, 1e-10);
	double a_Tz[3];
	s_mm(3, 1, 3, pinv_tcf, oz_vec, a_Tz);
	for (int i = 0; i < 3; i++)
	{
		a_Tz[i] = a_Tz[i] / len_oz;
	}
	//ĩ��Flange����ϵ��Z�ᣨ0��0��1��������ת���Ϊa_Tz,��Fz��a_TzΪ��������ƽ��ķ�����
	double Fz[3] = { 0,0,1 };
	double s_Ty[3], n_Tx[3];
	double prod_oy = 0;
	double prod_ox = 0;
	//��˻��Y�᷽��������������λ��
	crossVector(a_Tz, Fz, s_Ty);
	for (int i = 0; i < 3; i++)
	{
		prod_oy = prod_oy + s_Ty[i] * s_Ty[i];
	}
	for (int i = 0; i < 3; i++)
	{
		s_Ty[i] = s_Ty[i] / std::sqrt(prod_oy);
	}
	//��˻��X�᷽��������������λ��
	crossVector(s_Ty, a_Tz, n_Tx);
	for (int i = 0; i < 3; i++)
	{
		prod_ox = prod_ox + n_Tx[i] * n_Tx[i];
	}
	for (int i = 0; i < 3; i++)
	{
		n_Tx[i] = n_Tx[i] / std::sqrt(prod_ox);
	}
	for (int i = 0; i < 3; i++)
	{
		tcf[i * 3 + 0] = n_Tx[i];
		tcf[i * 3 + 1] = s_Ty[i];
		tcf[i * 3 + 2] = a_Tz[i];
	}
	return 0;
}
auto CalibT5P::tm2RP_5Pt(double tm[80], double *R, double *P)->int
{
	//��n_pt*16����任��������ȡn_pt*9��ת�����n_pt*3ƽ������
	int num = 80;
	int n_pt = 5;
	double temp[5][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT5P::deltaRP_5Pt(double R[45], double P[15], double * deltaR, double * deltaP)->int
{
	//����R(i)-R(i+1),P(i+1)-P(i),i=ʾ�̵�����n_pt��1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//6�㷨�궨��������ϵ
struct CalibT6PParam
{
	double pe_6pt[36];
	std::vector<double> tool_pe;
	std::string calib_info;
	std::string tool_name;
};
auto CalibT6P::prepareNrt()->void
{
	//������ʼ������ȡ��ǰʾ�̵�λ������
	CalibT6PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr = std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_6pt, 36);
			if (ret1 != 0) return;
		}
		else if (p.first == "tool")
		{
			param.tool_name = std::string(p.second);
		}
	}
	param.tool_pe.resize(6,0.0);
	this->param() = param;
	double pm_6pt[96];
	double tcp[3];		//�����õ�tcp
	double tcp_error;	//����tcp�����
	for (int i = 0; i < 6; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_6pt[6 * i + j];
		}
        s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_6pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret = cal_TCP_TCF(pm_6pt, tcp, tcp_error, param.tool_pe.data());//����313ŷ������ʽ�Ĺ�������ϵ
	if (ret == 0)
	{
		try
		{
			model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->setPrtPe(param.tool_pe.data());
		}
		catch (std::exception)
		{
			std::string calib_info = "cann't find \"" + param.tool_name + "\" node in partPool.";
			throw std::runtime_error("cann't find \"" + param.tool_name + "\" node in partPool.");
		}

		const std::string calib_info = "The configuration node of " + param.tool_name + "'s pose is created or updated.";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("The calculation process was aborted!");		//�޷�����궨�������ȡ��ʾ�̵��쳣��������ִ�б궨���̡�
		const std::string calib_info = "The calculation process was aborted!";
		param.calib_info = calib_info.c_str();
	}

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	this->ret() = out_param;

    option() |= NOT_RUN_EXECUTE_FUNCTION;
}
CalibT6P::CalibT6P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"CalibT6P\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}\"/>"
		"		<Param name=\"tool\" default=\"tool1\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT6P::cal_TCP_TCF(double transmatric[96], double tcp[3], double &tcp_error, double tcf[9])->int
{
	//����R(i)-R(i+1),P(i+1)-P(i)
	double R[54], P[18];
	tm2RP_6Pt(transmatric, R, P);
	double tcp_R[36], tcp_P[12], tcf_R[27], tcf_P[9];
	//��ȡ����tcp������
	for (int i = 0; i < 36; i++)
	{
		tcp_R[i] = R[i];
	}
	for (int i = 0; i < 12; i++)
	{
		tcp_P[i] = P[i];
	}
	//��ȡ����tcf������
	for (int i = 0; i < 27; i++)
	{
		tcf_R[i] = R[54 - 27 + i];
	}
	for (int i = 0; i < 9; i++)
	{
		tcf_P[i] = P[18 - 9 + i];
	}

	//����tcp
	double deltaR[27], deltaP[9];
	deltaRP_6Pt(tcp_R, tcp_P, deltaR, deltaP);
	double U[27], tau[9];
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	if (rank < 3)
	{
		//throw std::runtime_error("ʾ�̵������쳣�������»�ȡʾ�̵㣡");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//����tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);//���tcp��Է�����λ��
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
	}
	double prod_error[3] = { 0 };
	double sum_prod_error = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);

	//����tcf
	/*
	double tcf_deltaRz[9], tcf_deltaPz[3], tcf_deltaRy[9], tcf_deltaPy[3];
	for (int i = 0; i < 9; i++)
	{
		tcf_deltaRz[i] = tcf_R[9 + i] - tcf_R[i];
		tcf_deltaRy[i] = tcf_R[18 + i] - tcf_R[i];
	}
	for (int i = 0; i < 3; i++)
	{
		tcf_deltaPz[i] = tcf_P[i + 3] - tcf_P[i];
		tcf_deltaPy[i] = tcf_P[i + 6] - tcf_P[i];
	}
	double temp_vecZ[3], oz_vec[3];
	double temp_vecY[3], oy_vec[3];
	s_mm(3, 1, 3, tcf_deltaRz, tcp, temp_vecZ);
	s_mm(3, 1, 3, tcf_deltaRy, tcp, temp_vecY);
	for (int i = 0; i < 3; i++)
	{
		oz_vec[i] = temp_vecZ[i] + tcf_deltaPz[i];
		oy_vec[i] = temp_vecY[i] + tcf_deltaPy[i];
	}
	double prod_oz = 0;
	double len_oz = 0;
	double prod_oy = 0;
	double len_oy = 0;
	for (int i = 0; i < 3; i++)
	{
		prod_oz = prod_oz + oz_vec[i] * oz_vec[i];
		prod_oy = prod_oy + oy_vec[i] * oy_vec[i];
	}
	len_oz = std::sqrt(prod_oz);
	len_oy = std::sqrt(prod_oy);
	double tcf_R0[9];
	for (int i = 0; i < 9; i++)
	{
		tcf_R0[i] = tcf_R[i];
	}
	double U_tcf[9], tau_tcf[3];
	aris::Size p_tcf[3];
	aris::Size rank_tcf;
	s_householder_utp(3, 3, tcf_R0, U_tcf, tau_tcf, p_tcf, rank_tcf, 1e-10);
	double tau2_tcf[3], pinv_tcf[9];
	s_householder_utp2pinv(3, 3, rank_tcf, U_tcf, tau_tcf, p_tcf, pinv_tcf, tau2_tcf, 1e-10);
	double s_Ty[3], a_Tz[3];
	s_mm(3, 1, 3, pinv_tcf, oz_vec, a_Tz);
	s_mm(3, 1, 3, pinv_tcf, oy_vec, s_Ty);
	for (int i = 0; i < 3; i++)
	{
		a_Tz[i] = a_Tz[i] / len_oz;
		s_Ty[i] = s_Ty[i] / len_oy;
	}
	double n_Tx[3];
	double prod_ox = 0;
	//��˻��X�᷽��������������λ��
	crossVector(s_Ty, a_Tz, n_Tx);
	for (int i = 0; i < 3; i++)
	{
		prod_ox = prod_ox + n_Tx[i] * n_Tx[i];
	}
	for (int i = 0; i < 3; i++)
	{
		n_Tx[i] = n_Tx[i] / std::sqrt(prod_ox);
	}
	for (int i = 0; i < 3; i++)
	{
		tcf[i * 3 + 0] = n_Tx[i];
		tcf[i * 3 + 1] = s_Ty[i];
		tcf[i * 3 + 2] = a_Tz[i];
	}
	*/
	double zero[3] = { tcp[0], tcp[1], tcp[2] };//��������ϵԭ��
	double pm1[16];
	std::copy(transmatric, transmatric + 16, pm1);//��ȡ��һ�����λ�˾���,�����������base�ľ���
	double pg[3]{pm1[3], pm1[7], pm1[11]};//��ȡ��һ�����λ�ã������������base��λ��
	s_mma(3, 1, 3, pm1, 4, zero, 1, pg, 1);//��ȡ�ο�������base��λ��pg

	double *pm5 = transmatric + 64;//��ȡ��5�����λ�˾���,�����������base�ľ���
	double pt5[3]{ pm5[3], pm5[7], pm5[11] };//��ȡ��5�����λ�ã������������base��λ��
	s_mma(3, 1, 3, pm5, 4, zero, 1, pt5, 1);//��ȡ�ڵ�5��ʾ�̵�ʱ��tcp���base��λ��pt5

	double x_base[3] = { pg[0] - pt5[0], pg[1] - pt5[1], pg[2] - pt5[2] };//��ù�������ϵx�����base������
	double x_raw[3];
	s_mm(3, 1, 3, pm5, aris::dynamic::ColMajor{ 4 }, x_base, 1, x_raw, 1);//��ù�������ϵx������
	
	double pm6[16];
	std::copy(transmatric + 80, transmatric + 96, pm6);//��ȡ��6�����λ�˾���,�����������base�ľ���
	double pt6[3]{ pm6[3], pm6[7], pm6[11] };//��ȡ��6�����λ�ã������������base��λ��
	s_mma(3, 1, 3, pm6, 4, zero, 1, pt6, 1);//��ȡ�ڵ�6��ʾ�̵�ʱ��tcp���base��λ��pt6

	double y_base[3] = { pg[0] - pt6[0], pg[1] - pt6[1], pg[2] - pt6[2] };//��ù�������ϵy����·������base������
	double xy_raw[3];
	s_mm(3, 1, 3, pm6, aris::dynamic::ColMajor{ 4 }, y_base, 1, xy_raw, 1);//��ù�������ϵxy������
	
	double y_raw[3], z_raw[3];

	auto norm_x = aris::dynamic::s_norm(3, x_raw);
	if (std::abs(norm_x) < 1e-9)
	{
		return -1;
	}
	double x[3] = { x_raw[0] / norm_x, x_raw[1] / norm_x, x_raw[2] / norm_x };//��������ϵx�᷽������

	crossVector(x_raw, xy_raw, z_raw);
	auto norm_z = aris::dynamic::s_norm(3, z_raw);
	if (std::abs(norm_z) < 1e-9)
	{
		return -1;
	}
	double z[3] = { z_raw[0] / norm_z, z_raw[1] / norm_z,  z_raw[2] / norm_z };//��������ϵz�᷽������

	crossVector(z_raw, x_raw, y_raw);
	auto norm_y = aris::dynamic::s_norm(3, y_raw);
	if (std::abs(norm_y) < 1e-9)
	{
		return -1;
	}
	double y[3] = { y_raw[0] / norm_y, y_raw[1] / norm_y,  y_raw[2] / norm_y };//��������ϵy�᷽������

	double tool_pm[16]{ x[0], y[0], z[0], zero[0], x[1], y[1], z[1], zero[1], x[2], y[2], z[2], zero[2], 0, 0, 0, 1 };

	//���궨���ת��Ϊ313ŷ������ʽ
	aris::dynamic::s_pm2pe(tool_pm, tcf, "313");
	/*
	double ground_pnt[3];
	aris::dynamic::s_pp2pp(pm1, tcf, ground_pnt);
	aris::dynamic::dsp(1, 3, ground_pnt);
	aris::dynamic::s_pp2pp(transmatric + 16, tcf, ground_pnt);
	aris::dynamic::dsp(1, 3, ground_pnt);
	aris::dynamic::s_pp2pp(transmatric + 32, tcf, ground_pnt);
	aris::dynamic::dsp(1, 3, ground_pnt);
	aris::dynamic::s_pp2pp(transmatric + 48, tcf, ground_pnt);
	aris::dynamic::dsp(1, 3, ground_pnt);

	auto &tool0_prt_pm = model()->partPool()[6].markerPool().findByName("tool0")->prtPm();
	double tool_final[16];
	aris::dynamic::s_mm(4, 4, 4, *tool0_prt_pm, tool_pm, tool_final);
	dsp(4, 4, tool_final);
	*/
	return 0;
}
auto CalibT6P::tm2RP_6Pt(double tm[96], double *R, double *P)->int
{
	//��n_pt*16����任��������ȡn_pt*9��ת�����n_pt*3ƽ������
	int num = 96;
	int n_pt = 6;
	double temp[6][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT6P::deltaRP_6Pt(double R[54], double P[18], double * deltaR, double * deltaP)->int
{
	//����R(i)-R(i+1),P(i+1)-P(i),i=ʾ�̵�����n_pt��1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//�趨��������ϵ�ı궨���
struct SetTFParam
{
	size_t tool_id;
	std::string tool_name;
	double tool_pe[6];
	std::string calib_info;
};
auto SetTF::prepareNrt()->void
{
	//������ʼ��
	SetTFParam param;
	std::vector<std::string> tempvec;
	for (auto &p : cmdParams())
	{
		if (p.first == "tool_id")
		{
			param.tool_id = int32Param(p.first);
		}
		else if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		else if (p.first == "tool_pe")
		{
			std::string tempstr=std::string(p.second);
			std::string posestr = tempstr.substr(tempstr.find("{") + 1, tempstr.find("}") - tempstr.find("{") - 1);
			std::string::size_type pos1 = 0;
			std::string::size_type pos2 = posestr.find(",");
			
			while (pos2 != std::string::npos)
			{
				tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
				pos1 = pos2 + 1;
				pos2 = posestr.find(",", pos1);
			}
			tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
		}
	}
	if (tempvec.size() != 6)
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong��");		//��������ϵλ�˵�������������
		param.calib_info = std::string("The input data of teaching point's pose is wrong��");
	}
	else
	{
		//��ȡ��������ϵ����ڷ�������ϵ��λ��
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe[i] = std::stod(tempvec.at(i));
		}
		double tool_pm_f[16];
		s_pe2pm(param.tool_pe, tool_pm_f, "321");
		//��ȡ��������ϵ����ڵ�������ϵ��λ��
		double tool0_pm_g[16];
		try
		{
			auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName("tool0")->prtPm();
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					tool0_pm_g[4 * i + j] = mat1[i][j];
				}
			}
		}
		catch (std::exception)
		{
			throw std::runtime_error("cann't find \"tool0\" node in partPool.");
		}
		
		//���㹤������ϵ����ڵ�������ϵ��λ��
		double tool_pm_g[16];
		double tool_pe_g[6];
		s_mm(4, 4, 4, tool0_pm_g, tool_pm_f, tool_pm_g);
		s_pm2pe(tool_pm_g, tool_pe_g, "313");
		try
		{
			model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->setPrtPe(tool_pe_g);
		}
		catch(std::exception)
		{
			throw std::runtime_error("cann't find \"" + param.tool_name + "\" node in partPool.");
		}

		const std::string calib_info = "The configuration node of " + param.tool_name +"'s pose is created or updated.";
		param.calib_info = calib_info.c_str();
	}
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
SetTF::SetTF(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"SetTF\">"
		"	<GroupParam>"
		"		<Param name=\"tool_name\" default=\"tool1\"/>"
		"		<Param name=\"tool_pe\" default=\"{0,0,0,0,0,0}\"/>"
		"   </GroupParam>"
		"</Command>");
}


//�״λ��޸������궨�����������޹��߸��ء����Ӹ��ؼ��ⲿ����ʱ���еı궨������ʱ�����������ֵ��Ϊ��׼ֵ��
//�޸������궨�빤���޹�
//�����˹���ʱ����һ�̶ֹ��򲨶���С�ĸ���ʱҲ���Բ��ô˷����������궨
struct CalibZFParam 
{
	size_t axis_id;		//��ǰ�궨�������ֵ0-5
	std::string calib_info;
};
double CalibZF::zeroVal[6] = { 0 };
auto CalibZF::prepareNrt()->void
{
	//������ʼ��
	CalibZFParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
	}
	////�ӿ����������ļ����ȡ���궨��ԭ�������λ��
	
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZF::zeroVal[i] = mat->data().data()[i];
		}
	}
	
	this->param() = param;

	//��ȡ��ǰ�궨�����λ��ֵ
	CalibZF::zeroVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp();
	
	//���µı궨�����λ��д������������ļ�
	aris::core::Matrix mat1(1, 6, CalibZF::zeroVal);
	//model()->variablePool().findByName("tool0_axis_home")

	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"))->data() = mat1;
	}
	else
	{
		model()->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", mat1);
	}
	
	const std::string calib_info = "The current axis's zero positon has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZF::CalibZF(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zf\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//�и����µ����ƫ�����궨���������Ͻ����ڲ����أ��磺���߸��ؼ��ؽ��ϵĸ��Ӹ���ʱ����õ��ǲ�ͬ���߶�Ӧ�����ƫ����������������״α궨��׼ֵ��������
//��������ҵʱ���ⲿ���ؽϴ�ʱ����Ҫ�Բ�ͬ���ⲿ���ؽ������ƫ�����궨��
//���ó���������������+ĩ��ִ�������ľ�ȷ��λ�����磺���ӡ������и��/ϳ������ĥ
struct CalibZOParam
{
	size_t axis_id;		//��ǰ�궨�������ֵ0-5
	std::string tool_name;		//��������
	//bool calib_finished_flag;		//�궨�Ƿ������־
	std::string calib_info;
};
//��̬������ʼ��
size_t CalibZO::axisNum = 0;		//�ѱ궨����������
double CalibZO::zeroVal[6] = { 0 };
double CalibZO::offsetVal[6] = { 0 };
auto CalibZO::prepareNrt()->void
{
	//������ʼ��
	CalibZOParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
		else if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		/*else if (p.first == "calib_finished_flag")
		{
			param.calib_finished_flag = int32Param(p.first);
		}*/
	}

	this->param() = param;
	////�ӿ����������ļ����ȡ���궨��ԭ������㼰��Ӧ���ߵ����ƫ����
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZO::zeroVal[i] = mat->data().data()[i];
		}
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
			for (int i = 0; i < 6; i++)
			{
				CalibZO::offsetVal[i] = mat1->data().data()[i];
			}
		}

		//���㵱ǰ�궨�����λƫ����
		CalibZO::offsetVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp() - CalibZO::zeroVal[param.axis_id];

		//���µĸ������λ��ƫ����д������������ļ�
		aris::core::Matrix mat2(1, 6, CalibZO::offsetVal);
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"))->data() = mat2;
		}
		else
		{
			model()->variablePool().add<aris::dynamic::MatrixVariable>(param.tool_name + "_axis_offset", mat2);
		}
		const std::string calib_info = "The current axis's zero offset value with tool has been updated.";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file,please conduct \"CalibZF\" command firstly.").c_str();
		//return;
	}

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZO::CalibZO(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zo\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//������ƫ���������ⲿ�����µ����궨�����������г��˹��߸��غ͹ؽ��ϸ��Ӹ���֮����ⲿ���أ�
//���øø����µĹؽ�λ��ֵ��ȥƫ������������ø��ص��״α궨��㣬�����޸��ص��״α궨��㣩
struct CalibZLParam
{
	size_t axis_id;		//��ǰ�궨�������ֵ0-5
	std::string tool_name;		//��������
	//bool calib_finished_flag;		//�궨�Ƿ������־
	std::string calib_info;
};
size_t CalibZL::axisNum = 0;		//�ѱ궨����������
double CalibZL::zeroVal[6] = { 0 };
double CalibZL::offsetVal[6] = { 0 };
auto CalibZL::prepareNrt()->void
{
	//������ʼ��
	CalibZLParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
		else if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		/*else if (p.first == "calib_finished_flag")
		{
			param.calib_finished_flag = int32Param(p.first);
		}*/
	}
	////�ӿ����������ļ����ȡ�ѱ궨����
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZL::zeroVal[i] = mat->data().data()[i];
		}
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file").c_str();
		//return;
	}
	if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
	{
		auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
		for (int i = 0; i < 6; i++)
		{
			CalibZL::offsetVal[i] = mat1->data().data()[i];
		}
	}
	else
	{
		const std::string name = "\"" + param.tool_name + "_axis_offset\"" + " is not exist in controller's configuration file";
		throw std::runtime_error(name);
	}
	this->param() = param;

	//�������ⲿ����ʱ��ǰ�궨�����λ
	CalibZL::zeroVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp() - CalibZL::offsetVal[param.axis_id];
	
	//�궨��ɺ��и���ʱ�µĸ������λ��д������������ļ�������ԭ���λ��
	aris::core::Matrix mat2(1, 6, CalibZL::zeroVal);
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"))->data() = mat2;
	}
	else
	{
		model()->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", mat2);
	}
	const std::string calib_info = "The current axis's zero position with tool and external load has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZL::CalibZL(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zl\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//���»�����ģ��
struct SwitchToolParam
{
	std::string tool_name;		//��������
	double zeroVal[6] = { 0 };
	double offsetVal[6] = { 0 };
	std::string calib_info;
};
auto SwitchTool::prepareNrt()->void
{
	//������ʼ��
	SwitchToolParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
	}
	//�ӿ����������ļ����ȡtool0����λ
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			param.zeroVal[i] = mat->data().data()[i];
		}
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file").c_str();
		//return;
	}
	//�ӿ����������ļ����ȡ��ǰ���ߵ���λƫ����
	if (param.tool_name == "tool0")
	{
		for (int i = 0; i < 6; i++)
		{
			param.offsetVal[i] = 0;
		}
	}
	else
	{
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
			for (int i = 0; i < 6; i++)
			{
				param.offsetVal[i] = mat1->data().data()[i];
			}
		}
		else
		{
			const std::string name = "\"" + param.tool_name + "_axis_offset\"" + " is not exist in controller's configuration file";
			throw std::runtime_error(name);
		}
	}
	
	//���¿�������ģ���и������λ
	for (int i = 0; i < 6; i++)
	{
		controller()->motionPool()[i].setPosOffset(param.zeroVal[i] + param.offsetVal[i]);
	}
	//����ģ���еĹ���
	/*
	double tool_pm_g[16];
	try
	{
		auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->prtPm();
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				tool_pm_g[4 * i + j] = mat1[i][j];
			}
		}
		//����ģ�͵Ĺ�������ϵ//

	}
	catch (std::exception)
	{
		throw std::runtime_error(param.tool_name + " node is not in partPool.");
	}
	*/
	const std::string calib_info = "The tool of robot model has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
SwitchTool::SwitchTool(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"switchtool\">"
		"	<GroupParam>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}
