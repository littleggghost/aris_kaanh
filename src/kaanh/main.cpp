#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::dynamic;

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
const std::string modelxmlfile = "model_rokae.xml";
const std::string logfolder = "log";
std::thread t_modbus;


int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
	uixmlpath = uixmlpath / uixmlfile;
	modelxmlpath = modelxmlpath / modelxmlfile;
	logpath = logpath / logfolder;
    
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc < 2 ? xmlpath : argv[2];
	auto logp = argc < 2 ? logpath : argv[3];

	std::cout << "port:" << port << std::endl;
    std::cout << "xmlpath:" << xmlpath << std::endl;
    std::cout << "path:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

    /*
	//生成kaanh.xml文档
    //-------for rokae robot begin
    cs.resetController(kaanhconfig::createControllerRokaeXB4().release());
    cs.resetModel(kaanhconfig::createModelRokae().release());
    cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	//cs.interfacePool().add<kaanh::ProInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
    cs.resetSensorRoot(new aris::sensor::SensorRoot);
	//cs.interfaceRoot().loadXmlFile(uixmlpath.string().c_str());
	//cs.model().saveXmlFile(modelxmlpath.string().c_str());	//when creat new model
    cs.model().loadXmlFile(modelxmlpath.string().c_str());
    cs.saveXmlFile(xmlpath.string().c_str());
    //-------for rokae robot end// 
    */
    
    /*
	auto ret_load = cal.calculateExpression("pose({1,2,3,4,5,6,7})");
	std::cout << ret_load.first << std::endl;
	auto mat = std::any_cast<aris::core::Matrix>(ret_load.second);
	std::cout << mat.data()[0] << std::endl;

	cal.addVariable("p10", "pose", aris::core::Matrix({ 1,2,3,4,5,6,7 }));
	auto ret_ff = cal.calculateExpression("pose(p10)");
	std::cout << ret_ff.first << std::endl;
	auto ret_ff2 = cal.calculateExpression("pose({6,5,4,3,2,1,0})");
	std::cout << ret_ff2.first << std::endl;
    */

    /*
	//构造一个类型来接收UI变量//
	auto &cal = cs.model().calculator();
	cal.addTypename("load");
	cal.addFunction("load", std::vector<std::string>{"Matrix"}, "load", [](std::vector<std::any>&params)->std::any 
	{
		if (std::any_cast<aris::core::Matrix>(params[0]).size() != 11)
		{
			THROW_FILE_LINE("input data error");
		}
		Load a;
		a.mass = std::any_cast<aris::core::Matrix>(params[0]).data()[0];
		auto temp = std::any_cast<aris::core::Matrix&>(params[0]).data();
		std::copy(temp + 1, temp + 11, a.pq);

		return a;
	});
	auto ret_load = cal.calculateExpression("load({1,2,3,4,5,6,7,8,9,0,1})");
	std::cout << ret_load.first << std::endl;
	auto mat = std::any_cast<Load>(ret_load.second);
	cal.calculateExpression("load({2,3,4,5,6,7,8,9,0,1})");
	*/

	/*
	g_cal.addVariable("tool_pq", "Matrix", aris::core::Matrix(1.0));
	g_cal.addVariable("test", "String", std::string("1121"));
	//({ tool_pq,0.3 }*0.5 + 0.1) + 0.1;
	auto ret_mat = std::any_cast<aris::core::Matrix>(g_cal.calculateExpression("tool_pq").second);
	auto is_true = std::any_cast<std::string>(g_cal.calculateExpression("test").second);
	std::cout << ret_mat.toString() << std::endl;
	std::cout << is_true << std::endl;
	*/


    cs.loadXmlFile(path.string().c_str());
    //cs.resetPlanRoot(kaanhconfig::createPlanRoot().release());
    //cs.saveXmlFile(xmlpath.string().c_str());
	cs.init();

	/*test*/
	/*
	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3, 0.2, 0.6, 1.57, 0.8, 1.57}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp() / 2 / PI * 360.0 <<"   ";
	std::cout << std::endl;

	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3, 0.2, 0.6, 1.27, 0.8, 1.57}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp()/2/PI*360.0 << "   ";
	std::cout << std::endl;

	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3, 0.2, 0.6, 1.27, 0.8, 1.27}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp() / 2 / PI * 360.0 << "   ";
	std::cout << std::endl;

	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3, 0.2, 0.6, 0.9, 1.2, 1.2}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp() / 2 / PI * 360.0 << "   ";
	std::cout << std::endl;
	// - 0.68221807128202   0.73009129686273   0.30000000000000
	//	0.49378190310898 - 0.64847195478769 - 0.57936478665510   0.20000000000000
	//	0.86869685777062   0.33773159027558   0.36235775447668   0.60000000000000
	//	0.00000000000000   0.00000000000000   0.00000000000000   1.00000000000000
	dsp(4, 4, *cs.model().partPool()[6].markerPool().findByName("tool1")->pm());
	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3 + 0.03930905063127*0.1, 0.2- 0.49378190310898*0.1, 0.6- 0.86869685777062*0.1, 0.9, 1.2, 1.2}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp() / 2 / PI * 360.0 << "   ";
	std::cout << std::endl;

	cs.model().partPool()[6].markerPool().findByName("tool1")->setPe(std::array<double, 6>{0.3 + 0.68221807128202*0.1, 0.2 + 0.64847195478769*0.1, 0.6 - 0.33773159027558*0.1, 0.9, 1.2, 1.2}.data());
	cs.model().generalMotionPool()[0].updMpm();
	cs.model().solverPool()[0].kinPos();
	for (int i = 0; i < 6; ++i)std::cout << cs.model().motionPool()[i].mp() / 2 / PI * 360.0 << "   ";
	std::cout << std::endl;
	*/

	aris::core::logDirectory(logp);

    auto &cal = cs.model().calculator();
    kaanhconfig::createUserDataType(cal);
	kaanhconfig::createPauseTimeSpeed();
    //cs.start();

	//实时回调函数，每个实时周期调用一次//
	cs.setRtPlanPostCallback(kaanh::update_state);
	g_model = cs.model();

#ifdef WIN32
	for (auto &m : cs.controller().motionPool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif // WIN32

	//Start Web Socket//
    cs.open();

    //示教器线程;
    t_modbus = std::thread([&]()->bool
    {
       // 创建modbus master
       modbus mb = modbus("192.168.0.21", 502);	//从站的ip地址和端口号
       mb.modbus_set_slave_id(1);					// set slave id
       controlboard cbd;

       // 连接modbus slave，并且响应modbus slave的信息
   start:
       for (;;)
       {
           try
           {
               mb.modbus_connect();				// connect with the server
               break;
           }
           catch (std::exception &e)
           {
               std::cout << "failed to connect server, will retry in 1 second" << std::endl;
               std::this_thread::sleep_for(std::chrono::seconds(1));
           }
       }

       // 主要功能逻辑区
       try
       {
           cs.executeCmd("setvel --vel_percent=1;");

   #if print_time
           struct timeb t1;
           double time_new, time_last, time_max = 0;
           ftime(&t1);
           time_new = time_last = t1.time + 0.001* t1.millitm;
   #endif

           uint16_t read_input_regs[2];
           while (1)
           {
   #if print_time
               ftime(&t1);
               time_last = time_new;
               time_new = t1.time + 0.001* t1.millitm;
               if (time_new - time_last > time_max)
                   time_max = time_new - time_last;
               printf("[%.3f %.3f %.3f]\n", time_new, time_new - time_last, time_max);
   #endif

               /*************以上打印时间相关**************************/
               //read_input_regs[0]存地址为30001的寄存器,read_input_regs[1]存地址为30002的寄存器，读输入寄存器 功能码04
               mb.modbus_read_input_registers(0, 2, read_input_regs);
               if (read_input_regs[0] != 0 || read_input_regs[1] != 0)			//有按键按下
                   cbd.send_command(read_input_regs, cs);					//发送指令
               std::this_thread::sleep_for(std::chrono::milliseconds(50));	//100ms
           }

       }
       catch (std::exception &e)
       {
           std::cout << e.what() << std::endl;
       }

       mb.modbus_close();	// close connection

       goto start;	// 断连接后，尝试重新连接

       delete(&mb); // 释放空间

       return 0 ;
    });

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
