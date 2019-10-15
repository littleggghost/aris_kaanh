﻿#include <algorithm>
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
        std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

        //force sensor//
        std::string xml_str =
            "<EthercatSlave phy_id=\"0\" product_code=\"0x000c7011\""
            " vendor_id=\"0x000fffff\" revision_num=\"0x00000001\" dc_assign_activate=\"0x300\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1601\" is_tx=\"false\">"
            "				<PdoEntry name=\"led_1\" index=\"0x7010\" subindex=\"0x01\" size=\"1\"/>"
            "				<PdoEntry name=\"led_2\" index=\"0x7010\" subindex=\"0x02\" size=\"1\"/>"
            "				<PdoEntry name=\"led_3\" index=\"0x7010\" subindex=\"0x03\" size=\"1\"/>"
            "				<PdoEntry name=\"led_4\" index=\"0x7010\" subindex=\"0x04\" size=\"1\"/>"
            "				<PdoEntry name=\"led_5\" index=\"0x7010\" subindex=\"0x05\" size=\"1\"/>"
            "				<PdoEntry name=\"led_6\" index=\"0x7010\" subindex=\"0x06\" size=\"1\"/>"
            "				<PdoEntry name=\"led_7\" index=\"0x7010\" subindex=\"0x07\" size=\"1\"/>"
            "				<PdoEntry name=\"led_8\" index=\"0x7010\" subindex=\"0x08\" size=\"1\"/>"
            "				<PdoEntry name=\"entryname\" index=\"0x0000\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"switch_1\" index=\"0x6000\" subindex=\"0x01\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_2\" index=\"0x6000\" subindex=\"0x02\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_3\" index=\"0x6000\" subindex=\"0x03\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_4\" index=\"0x6000\" subindex=\"0x04\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_5\" index=\"0x6000\" subindex=\"0x05\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_6\" index=\"0x6000\" subindex=\"0x06\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_7\" index=\"0x6000\" subindex=\"0x07\" size=\"1\"/>"
            "				<PdoEntry name=\"switch_8\" index=\"0x6000\" subindex=\"0x08\" size=\"1\"/>"
            "				<PdoEntry name=\"entryname\" index=\"0x0000\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "			<Pdo index=\"0x1A02\" is_tx=\"true\">"
            "				<PdoEntry name=\"underrange\" index=\"0x6020\" subindex=\"0x01\" size=\"1\"/>"
            "				<PdoEntry name=\"overrange\" index=\"0x6020\" subindex=\"0x02\" size=\"1\"/>"
            "				<PdoEntry name=\"limit_1\" index=\"0x6020\" subindex=\"0x03\" size=\"2\"/>"
            "				<PdoEntry name=\"limit_2\" index=\"0x6020\" subindex=\"0x05\" size=\"2\"/>"
            "				<PdoEntry name=\"entryname\" index=\"0x0000\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"txpdo_state\" index=\"0x1802\" subindex=\"0x07\" size=\"1\"/>"
            "				<PdoEntry name=\"txpdo_toggle\" index=\"0x1802\" subindex=\"0x09\" size=\"1\"/>"
            "				<PdoEntry name=\"fx\" index=\"0x6020\" subindex=\"0x11\" size=\"32\"/>"
            "				<PdoEntry name=\"fy\" index=\"0x6020\" subindex=\"0x12\" size=\"32\"/>"
            "				<PdoEntry name=\"fz\" index=\"0x6020\" subindex=\"0x13\" size=\"32\"/>"
            "				<PdoEntry name=\"mx\" index=\"0x6020\" subindex=\"0x14\" size=\"32\"/>"
            "				<PdoEntry name=\"my\" index=\"0x6020\" subindex=\"0x15\" size=\"32\"/>"
            "				<PdoEntry name=\"mz\" index=\"0x6020\" subindex=\"0x16\" size=\"32\"/>"
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
        dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x00);
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
          std::vector<std::pair<std::string, std::any>> ret;
          target.ret = ret;

      }
      auto FSSignal::executeRT(PlanTarget &target)->int
          {
              auto &param = std::any_cast<FSParam&>(target.param);
              // 访问主站 //
              auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
              int fx, fy, fz, mx, my, mz;
              bool is_true = true;
              bool is_false = false;


              if(target.count == 1)
              {
                  controller->slavePool().at(0).writePdo(0x7010, 0x01, &is_true, 1);
                  controller->slavePool().at(0).writePdo(0x7010, 0x02, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x03, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x04, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x05, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x06, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x07, &is_false, 1);
                  //controller->slavePool().at(0).writePdo(0x7010, 0x08, &is_false, 1);
              }

              /*
              controller->slavePool().at(0).readPdo(0x6020, 0x11, &fx ,32);
              controller->slavePool().at(0).readPdo(0x6020, 0x12, &fy, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x13, &fz, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x14, &mx, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x15, &my, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x16, &mz, 32);
              */
              controller->slavePool().at(0).readPdo(0x6020, 0x11, &param.Fx ,32);
              controller->slavePool().at(0).readPdo(0x6020, 0x12, &param.Fy, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x13, &param.Fz, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x14, &param.Mx, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x15, &param.My, 32);
              controller->slavePool().at(0).readPdo(0x6020, 0x16, &param.Mz, 32);

              //print//
              auto &cout = controller->mout();
              if (target.count % 100 == 0)
              {
                  /*
                  cout << std::setw(6) << fx << "  ";
                  cout << std::setw(6) << fy << "  ";
                  cout << std::setw(6) << fz << "  ";
                  cout << std::setw(6) << mx << "  ";
                  cout << std::setw(6) << my << "  ";
                  cout << std::setw(6) << mz << "  ";
                  */

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
