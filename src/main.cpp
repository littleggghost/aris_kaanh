﻿#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


std::atomic_bool is_automatic = false;
using namespace aris::dynamic;

//global vel//
kaanh::Speed g_vel;
std::atomic_int g_vel_percent = 0;
//global vel//

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
//for qifan robot//
//const std::string modelxmlfile = "model_qifan.xml";
const std::string modelxmlfile = "model_rokae.xml";



int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	modelxmlpath = modelxmlpath / modelxmlfile;
    std::cout<< xmlpath <<std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

	/*
	//生成kaanh.xml文档	
	//-------for qifan robot begin//
	cs.resetController(kaanh::createControllerQifan().release());
	cs.resetModel(kaanh::createModelQifan().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for qifan robot end// 
	*/

	
	//-------for rokae robot begin//
	cs.resetController(kaanh::createControllerRokaeXB4().release());
	cs.resetModel(kaanh::createModelRokae().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
    cs.model().loadXmlFile(modelxmlpath.string().c_str());
	cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for rokae robot end// 
	

	cs.loadXmlFile(xmlpath.string().c_str());
	
	cs.start();

	//加载v100的速度值//
	auto &getspeed = dynamic_cast<aris::dynamic::MatrixVariable &>(*cs.model().variablePool().findByName("v100"));
	kaanh::SpeedParam speed;
	std::copy(getspeed.data().begin(), getspeed.data().end(), &speed.w_percent);
	speed.w_tcp = speed.w_tcp * speed.w_percent;
	g_vel.setspeed(speed);

	//Start Web Socket//
    cs.open();

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
