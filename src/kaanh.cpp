#include <algorithm>
#include "kaanh.h"
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>


using namespace aris::dynamic;
using namespace aris::plan;


namespace kaanh
{
    auto createController()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
    {
        std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
        controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加

        //ATI force sensor//
        std::string xml_str =
            "<EthercatSlave phy_id=\"0\" product_code=\"0x000c7011\""
            " vendor_id=\"1048575\" revision_num=\"0x00000001\" dc_assign_activate=\"0x00\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1601\" is_tx=\"false\">"
            "				<PdoEntry name=\"Control_1\" index=\"0x7010\" subindex=\"0x01\" size=\"32\"/>"
            "				<PdoEntry name=\"Control_2\" index=\"0x7010\" subindex=\"0x02\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"Int_Input_Fx\" index=\"0x6000\" subindex=\"0x01\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Fy\" index=\"0x6000\" subindex=\"0x02\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Fz\" index=\"0x6000\" subindex=\"0x03\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Mx\" index=\"0x6000\" subindex=\"0x04\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_My\" index=\"0x6000\" subindex=\"0x05\" size=\"32\"/>"
            "				<PdoEntry name=\"Int_Input_Mz\" index=\"0x6000\" subindex=\"0x06\" size=\"32\"/>"
            "				<PdoEntry name=\"Status_Code\" index=\"0x6010\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatSlave>";
        controller->slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);

/*
        controller->slavePool().add<aris::control::EthercatSlave>();
        controller->slavePool().back().setPhyId(0);
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x300);
*/
        return controller;
    }

    // 力传感器信号测试 //
      struct FSParam
      {
          bool real_data;
          int time;
          uint16_t datanum;
          float Fx,Fy,Fz,Mx,My,Mz;
      };
      auto FSSignal::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
          {
              FSParam param;
              for (auto &p : params)
              {
                  if (p.first == "real_data")
                  {
                      param.real_data = std::stod(p.second);
                  }
                  else if (p.first == "time")
                  {
                      param.time = std::stoi(p.second);
                  }
              }

              param.Fx = 0.0;
              param.Fy = 0.0;
              param.Fz = 0.0;
              param.Mx = 0.0;
              param.My = 0.0;
              param.Mz = 0.0;
              target.param = param;

  #ifdef WIN32
              target.option |=

                  Plan::NOT_CHECK_POS_MIN |
                  Plan::NOT_CHECK_POS_MAX |
                  Plan::NOT_CHECK_POS_CONTINUOUS |
                  Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
                  Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                  Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
                  Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
                  Plan::NOT_CHECK_VEL_MIN |
                  Plan::NOT_CHECK_VEL_MAX |
                  Plan::NOT_CHECK_VEL_CONTINUOUS |
                  Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
                  Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
  #endif
          }
      auto FSSignal::executeRT(PlanTarget &target)->int
          {
              auto &param = std::any_cast<FSParam&>(target.param);
              // 访问主站 //
              auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
              if (param.real_data)
              {
                  controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                  controller->slavePool().at(6).readPdo(0x6030, 0x01, &param.Fx ,32);
                  controller->slavePool().at(6).readPdo(0x6030, 0x02, &param.Fy, 32);
                  controller->slavePool().at(6).readPdo(0x6030, 0x03, &param.Fz, 32);
                  controller->slavePool().at(6).readPdo(0x6030, 0x04, &param.Mx, 32);
                  controller->slavePool().at(6).readPdo(0x6030, 0x05, &param.My, 32);
                  controller->slavePool().at(6).readPdo(0x6030, 0x06, &param.Mz, 32);
              }
              else
              {
                  controller->slavePool().at(6).readPdo(0x6030, 0x00, &param.datanum ,16);
                  controller->slavePool().at(6).readPdo(0x6020, 0x01, &param.Fx, 32);
                  controller->slavePool().at(6).readPdo(0x6020, 0x02, &param.Fy, 32);
                  controller->slavePool().at(6).readPdo(0x6020, 0x03, &param.Fz, 32);
                  controller->slavePool().at(6).readPdo(0x6020, 0x04, &param.Mx, 32);
                  controller->slavePool().at(6).readPdo(0x6020, 0x05, &param.My, 32);
                  controller->slavePool().at(6).readPdo(0x6020, 0x06, &param.Mz, 32);
              }

              //print//
              auto &cout = controller->mout();
              if (target.count % 100 == 0)
              {
                  cout << std::setw(6) << param.datanum << "  ";
                  cout << std::setw(6) << param.Fx << "  ";
                  cout << std::setw(6) << param.Fy << "  ";
                  cout << std::setw(6) << param.Fz << "  ";
                  cout << std::setw(6) << param.Mx << "  ";
                  cout << std::setw(6) << param.My << "  ";
                  cout << std::setw(6) << param.Mz << "  ";
                  cout << std::endl;
                  cout << "----------------------------------------------------" << std::endl;
              }

              //log//
              auto &lout = controller->lout();
              {
                  lout << param.Fx << " ";
                  lout << param.Fy << " ";
                  lout << param.Fz << " ";
                  lout << param.Mx << " ";
                  lout << param.My << " ";
                  lout << param.Mz << " ";
                  lout << std::endl;
              }
              param.time--;
              return param.time;
          }
      auto FSSignal::collectNrt(PlanTarget &target)->void {}
      FSSignal::FSSignal(const std::string &name) :Plan(name)
      {
          command().loadXmlStr(
              "<Command name=\"fssignal\">"
              "	<GroupParam>"
              "		<Param name=\"real_data\" default=\"1\"/>"
              "		<Param name=\"time\" default=\"100000\"/>"
              "	</GroupParam>"
              "</Command>");
      }


      // ATI Force Sensor //
      struct ATIFSParam
      {
          int time;
          std::int32_t Fx,Fy,Fz,Mx,My,Mz,status_code,sample_counter,controlcodes;
          std::int32_t forceindex, torqueindex;
          std::uint8_t forceunit, torqueunit;
      };
      auto ATIFS::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
          {
              auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
              ATIFSParam param;
              param.Fx = 0;
              param.Fy = 0;
              param.Fz = 0;
              param.Mx = 0;
              param.My = 0;
              param.Mz = 0;
              param.status_code = 0;
              param.sample_counter = 0;
              param.controlcodes = 4096;
              //param.controlcodes = 4352;//little range

              for (auto &p : params)
              {
                  if (p.first == "time")
                  {
                      param.time = std::stoi(p.second);
                  }
                  else if (p.first == "controlcodes")
                  {
                      param.controlcodes = std::stoi(p.second);
                  }
              }

              /*
              controller->slavePool().at(6).readSdo(0x2021, 0x2f, &param.forceunit, 8);
              controller->slavePool().at(6).readSdo(0x2021, 0x30, &param.torqueunit, 8);
              controller->slavePool().at(6).readSdo(0x2021, 0x37, &param.forceindex, 32);
              controller->slavePool().at(6).readSdo(0x2021, 0x38, &param.torqueindex, 32);
              */

              target.param = param;
              std::fill(target.mot_options.begin(), target.mot_options.end(),
                  Plan::NOT_CHECK_ENABLE);

          }
      auto ATIFS::executeRT(PlanTarget &target)->int
      {
          auto &param = std::any_cast<ATIFSParam&>(target.param);
          // 访问主站 //

          auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
          if (target.count == 1)
          {
              controller->slavePool().at(6).writePdo(0x7010, 0x01, &param.controlcodes, 32);
          }
          controller->slavePool().at(6).readPdo(0x6000, 0x01, &param.Fx ,32);
          controller->slavePool().at(6).readPdo(0x6000, 0x02, &param.Fy, 32);
          controller->slavePool().at(6).readPdo(0x6000, 0x03, &param.Fz, 32);
          controller->slavePool().at(6).readPdo(0x6000, 0x04, &param.Mx, 32);
          controller->slavePool().at(6).readPdo(0x6000, 0x05, &param.My, 32);
          controller->slavePool().at(6).readPdo(0x6000, 0x06, &param.Mz, 32);
          controller->slavePool().at(6).readPdo(0x6010, 0x00, &param.status_code, 32);
          controller->slavePool().at(6).readPdo(0x6020, 0x00, &param.sample_counter, 32);

          /*
          //print//
          auto &cout = controller->mout();
          if (target.count % 100 == 0)
          {
              cout << std::setw(6) << param.Fx << "  ";
              cout << std::setw(6) << param.Fy << "  ";
              cout << std::setw(6) << param.Fz << "  ";
              cout << std::setw(6) << param.Mx << "  ";
              cout << std::setw(6) << param.My << "  ";
              cout << std::setw(6) << param.Mz << "  ";
              cout << std::setw(6) << param.status_code << "  ";
              cout << std::setw(6) << param.sample_counter << "  ";
              cout << std::setw(6) << param.forceunit << "  ";
              cout << std::setw(6) << param.torqueunit<< "  ";
              cout << std::setw(6) << param.forceindex << "  ";
              cout << std::setw(6) << param.forceindex << "  ";
              cout << std::endl;
              cout << "----------------------------------------------------" << std::endl;
          }
          */


          double Fx = param.Fx/1000000.0;
          double Fy = param.Fy/1000000.0;
          double Fz = param.Fz/1000000.0;
          double Mx = param.Mx/1000000.0;
          double My = param.My/1000000.0;
          double Mz = param.Mz/1000000.0;


          //print//
          auto &cout = controller->mout();
          if (target.count % 100 == 0)
          {
              cout << std::setw(6) << Fx << "  ";
              cout << std::setw(6) << Fy << "  ";
              cout << std::setw(6) << Fz << "  ";
              cout << std::setw(6) << Mx << "  ";
              cout << std::setw(6) << My << "  ";
              cout << std::setw(6) << Mz << "  ";
              cout << std::setw(6) << param.status_code << "  ";
              cout << std::setw(6) << param.sample_counter << "  ";
              cout << std::endl;
              cout << "----------------------------------------------------" << std::endl;
          }

          //log//
          auto &lout = controller->lout();
          {
              lout << Fx << " ";
              lout << Fy << " ";
              lout << Fz << " ";
              lout << Mx << " ";
              lout << My << " ";
              lout << Mz << " ";
              lout << param.status_code << " ";
              lout << param.sample_counter << " ";
              lout << std::endl;
          }

          param.time--;
          return param.time;
      }
      auto ATIFS::collectNrt(PlanTarget &target)->void {}
      ATIFS::ATIFS(const std::string &name) :Plan(name)
      {
          command().loadXmlStr(
              "<Command name=\"atifs\">"
              "	<GroupParam>"
              "		<Param name=\"time\" default=\"100000\"/>"
              "		<Param name=\"controlcodes\" default=\"4096\"/>"
              "	</GroupParam>"
              "</Command>");
      }


    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

        plan_root->planPool().add<kaanh::FSSignal>();
        plan_root->planPool().add<kaanh::ATIFS>();

		return plan_root;
	}

}
