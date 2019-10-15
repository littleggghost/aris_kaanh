#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::dynamic;


int main(int argc, char *argv[])
{
    std::cout <<"new"<<std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

    /*
    aris::control::EthercatMaster mst;
    mst.scan();
    std::cout << mst.slavePool().size() << std::endl;
    */

    cs.resetController(kaanh::createController().release());
    cs.resetPlanRoot(kaanh::createPlanRoot().release());
	cs.interfacePool().add<aris::server::WebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);

	cs.start();

	//Start Web Socket//
    cs.open();

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
