#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree{

    void Application::PrintMessageAll() {
        LoggerPtr->Debug("-------------PrintMessageAll-------------");
        LoggerPtr->Debug("|  Myself Hreo Position: {}, {}", friendRobots[UnitType::Hero].position_.X, friendRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Myself Engineer Position: {}, {}", friendRobots[UnitType::Engineer].position_.X, friendRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry1 Position: {}, {}", friendRobots[UnitType::Infantry1].position_.X, friendRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry2 Position: {}, {}", friendRobots[UnitType::Infantry2].position_.X, friendRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Myself Sentry Position: {}, {}", friendRobots[UnitType::Sentry].position_.X, friendRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Position: {}, {}", enemyRobots[UnitType::Hero].position_.X, enemyRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Enemy Engineer Position: {}, {}", enemyRobots[UnitType::Engineer].position_.X, enemyRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry1 Position: {}, {}", enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry2 Position: {}, {}", enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Enemy Sentry Position: {}, {}", enemyRobots[UnitType::Sentry].position_.X, enemyRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Myself Hero Health: {}", friendRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Myself Engineer Health: {}", friendRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry1 Health: {}", friendRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry2 Health: {}", friendRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Myself Sentry Health: {}", friendRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Health: {}", enemyRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Enemy Engineer Health: {}", enemyRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry1 Health: {}", enemyRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry2 Health: {}", enemyRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Enemy Sentry Health: {}", enemyRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|------------------------------------");
        LoggerPtr->Debug("|  DefenceBuff: {}", int(teamBuff.DefenceBuff));
        LoggerPtr->Debug("|  RecoveryBuff: {}", int(teamBuff.RecoveryBuff));
        LoggerPtr->Debug("|  RemainingEnergy: {}", int(teamBuff.RemainingEnergy));
        LoggerPtr->Debug("|  VulnerabilityBuff: {}", int(teamBuff.VulnerabilityBuff));
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  CapV: {}", int(capV));
        LoggerPtr->Debug("-----------End PrintMessageAll-----------");
    }
    void Application::SubscribeMessageAll() {
        // ly_gimbal_angles
        GenSub<ly_gimbal_angles>([](Application& app, auto msg) {
            app.gimbalAngles.Yaw = msg->Yaw;
            app.gimbalAngles.Pitch = msg->Pitch;
        });

        // ly_gimbal_firecode
        GenSub<ly_gimbal_firecode>([](Application& app, auto msg) {
            app.RecFireCode.FireStatus = (msg->data & 0b11);
        });

        // ly_gimbal_capV
        GenSub<ly_gimbal_capV>([](Application& app, auto msg) {
            app.capV = msg->data;
        });

        // ly_game_eventdata
        GenSub<ly_game_eventdata>([](Application& app, auto msg) {
            app.extEventData = msg->data;
        });

        // ly_me_is_team_red
        GenSub<ly_me_is_team_red>([](Application& app, auto msg) {
            app.team = msg->data ? UnitTeam::Red : UnitTeam::Blue;
        });

        // ly_game_all
        GenSub<ly_game_all>([](Application& app, auto msg) {
            app.myselfHealth = msg->SelfHealth;
        });

        // ly_enemy_op_hp
        GenSub<ly_enemy_op_hp>([](Application& app, auto msg) {
            app.enemyOutpostHealth = msg->data;
        });

        // ly_me_op_hp
        GenSub<ly_me_op_hp>([](Application& app, auto msg) {
            app.selfOutpostHealth = msg->data;
        });

        // ly_me_base_hp
        GenSub<ly_me_base_hp>([](Application& app, auto msg) {
            app.selfBaseHealth = msg->data;
        });

        // ly_enemy_base_hp
        GenSub<ly_enemy_base_hp>([](Application& app, auto msg) {
            app.enemyBaseHealth = msg->data;
        });

        // ly_me_ammo_left
        GenSub<ly_me_ammo_left>([](Application& app, auto msg) {
            app.ammoLeft = msg->data;
        });

        // ly_game_time_left
        GenSub<ly_game_time_left>([](Application& app, auto msg) {
            app.timeLeft = msg->data;
        });

        // ly_game_is_start
        GenSub<ly_game_is_start>([](Application& app, auto msg) {
            app.is_game_begin = msg->data;
        });

        // ly_navi_vel
        GenSub<ly_navi_vel>([](Application& app, auto msg) {
            app.naviVelocity.X = msg->X;
            app.naviVelocity.Y = msg->Y;
        });

        // ly_navi_lower_head
        GenSub<ly_navi_lower_head>([](Application& app, auto msg) {
            app.naviLowerHead = msg->data;
        });

        /**
         *     LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::BuffData);
    LY_DEF_ROS_TOPIC(ly_me_rfid, "/ly/me/rfid", std_msgs::UInt32);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::PositionData);
         */

        // ly_team_buff
        GenSub<ly_team_buff>([](Application& app, auto msg) {
            app.teamBuff.RecoveryBuff = msg->RecoveryBuff;
            app.teamBuff.CoolingBuff = msg->CoolingBuff;
            app.teamBuff.DefenceBuff = msg->DefenceBuff;
            app.teamBuff.VulnerabilityBuff = msg->VulnerabilityBuff;
            app.teamBuff.AttackBuff = msg->AttackBuff;
            app.teamBuff.RemainingEnergy = msg->RemainingEnergy;
        });

        // ly_me_rfid
        GenSub<ly_me_rfid>([](Application& app, auto msg) {
            app.rfidStatus = msg->data;
        });

        // ly_position_data
        GenSub<ly_position_data>([](Application& app, auto msg) {
            int FriendCarId = msg->FriendCarId;
            app.friendRobots[FriendCarId].position_.X = msg->FriendX;
            app.friendRobots[FriendCarId].position_.Y = 1500 - msg->FriendY;
            int EnemyCarId = msg->EnemyCarId;
            EnemyCarId = EnemyCarId % 100;
            app.enemyRobots[EnemyCarId].position_.X = msg->EnemyX;
            app.enemyRobots[EnemyCarId].position_.Y = 1500 - msg->EnemyY;
        });

        // ly_detector_armors
        GenSub<ly_detector_armors>([](Application& app, auto msg) {
            auto &armorList = app.armorList;
            std::fill(armorList.begin(), armorList.end(), ArmorData{ArmorType::UnKnown, 30});
            const auto &armors = msg->armors;
            int count = std::min(10, static_cast<int>(armors.size()));
            for (int i = 0; i < count; ++i) {
                armorList[i] = ArmorData{
                    static_cast<ArmorType>(armors[i].type),
                    armors[i].distance
                };
            }
        });

        // ly_predictor_target
        GenSub<ly_predictor_target>([](Application& app, auto msg) {
            auto &obj = app;
            obj.autoAimData.Angles.Yaw = msg->yaw;
            obj.autoAimData.Angles.Pitch = msg->pitch;
            obj.autoAimData.BuffFollow = false;
            obj.autoAimData.FireStatus = true;
            obj.isFindTargetAtomic = true;
            obj.LoggerPtr->Info("进入回调，更新角度");
        });

        // ly_buff_target
        GenSub<ly_buff_target>([](Application& app, auto msg) { 
            auto  &obj = app;
            obj.buffAimData.Angles.Pitch = msg->pitch;
            obj.buffAimData.Angles.Yaw = msg->yaw;
            obj.buffAimData.FireStatus = msg->status;
            obj.buffAimData.BuffFollow = true;
            // if(obj.buffAimData.FireStatus) {
            //     obj.isFindTargetAtomic = true;
            // }
            obj.isFindTargetAtomic = true;
        });

        // ly_outpost_target
        GenSub<ly_outpost_target>([](Application& app, auto msg) {
            auto &obj = app;
            obj.outpostAimData.Angles.Yaw = msg->yaw;
            obj.outpostAimData.Angles.Pitch = msg->pitch;
            obj.outpostAimData.FireStatus = true;
            obj.outpostAimData.BuffFollow = false;
            obj.isFindTargetAtomic = true;
        });

        // ly_enemy_hp
        GenSub<ly_enemy_hp>([](Application& app, auto msg) {
            app.enemyRobots[UnitType::Hero].setCurrentHealth(static_cast<std::uint16_t>(msg->hero));
            app.enemyRobots[UnitType::Engineer].setCurrentHealth(static_cast<std::uint16_t>(msg->engineer));
            app.enemyRobots[UnitType::Infantry1].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry1));
            app.enemyRobots[UnitType::Infantry2].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry2));
            app.enemyRobots[UnitType::Sentry].setCurrentHealth(static_cast<std::uint16_t>(msg->sentry));
        });

        // ly_me_hp
        GenSub<ly_me_hp>([](Application& app, auto msg) {
            app.friendRobots[UnitType::Hero].setCurrentHealth(static_cast<std::uint16_t>(msg->hero));
            app.friendRobots[UnitType::Engineer].setCurrentHealth(static_cast<std::uint16_t>(msg->engineer));
            app.friendRobots[UnitType::Infantry1].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry1));
            app.friendRobots[UnitType::Infantry2].setCurrentHealth(static_cast<std::uint16_t>(msg->infantry2));
            app.friendRobots[UnitType::Sentry].setCurrentHealth(static_cast<std::uint16_t>(msg->sentry));
        });
    }    

}