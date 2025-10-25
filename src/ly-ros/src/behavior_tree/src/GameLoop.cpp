#include "../include/Application.hpp"

using namespace LangYa;

namespace BehaviorTree {


    float normalize_angle_0_360(float angle) {
        float normalized = fmod(angle, 360.0f);
        if (normalized < 0)
            normalized += 360.0f;
        return normalized;
    }

     /**
     * @brief 更新黑板数据 \n
     * @brief  更新数据从上到下依次是：我方颜色，敌方哨站血量，我方哨站血量，剩余弹药，比赛剩余时间 \n
     * @brief  自身血量，己方英雄血量，己方3号步兵血量，视野中的装甲板序列，是否找到目标 \n
     */
    void Application::UpdateBlackBoard() {

        std::uint16_t SelfHealth = myselfHealth;
        bool IsFindTarget = isFindTargetAtomic;

        // 将数据写入黑板
        BlackBoard->set<UnitTeam>("MyTeam", team);
        BlackBoard->set<std::uint16_t>("TimeLeft", timeLeft);
        BlackBoard->set<std::uint16_t>("SelfHealth", SelfHealth);
        BlackBoard->set<std::uint16_t>("AmmoLeft", ammoLeft);
        BlackBoard->set<Robots>("FriendRobots", friendRobots);
        BlackBoard->set<Robots>("EnemyRobots", enemyRobots);
        BlackBoard->set<std::uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
        BlackBoard->set<std::uint16_t>("SelfOutpostHealth", selfOutpostHealth);
        BlackBoard->set("ArmorList", armorList);
        BlackBoard->set("TeamBuff", teamBuff);
        
        LoggerPtr->Info("-------- update blackboard ----------");
        LoggerPtr->Info("TimeLeft: {}", timeLeft);
        LoggerPtr->Info("SelfHealth: {}, AmmoLeft: {}", SelfHealth, ammoLeft);
        LoggerPtr->Info("EnemyOutpostHealth: {}, SelfOutpostHealth: {}", enemyOutpostHealth, selfOutpostHealth);
        LoggerPtr->Info("-------- update blackboard successfully --------");
    }

    /**
     * @brief 从黑板获取数据,处理ros队列的消息并发布 \n
     * @brief 从黑板获取的有辐瞄击打目标，导航目的地
     */
    void Application::TransportData() {
        // targetArmor.Type = GetInfoFromBlackBoard<ArmorType>("AimTarget");
        // naviCommandGoal = GetInfoFromBlackBoard<std::uint8_t>("naviCommandGoal");
        // LoggerPtr->Info("AimTarget: {}, NaviGoal: {}", static_cast<int>(targetArmor.Type), static_cast<int>(naviCommandGoal));

        
        // targetArmor = ArmorType::Infantry1;
        // 设置数据内容
        PublishTogether();
        PrintMessageAll();
        LoggerPtr->Info("send BlackBoard data successfully!");
    }

     /**
     * @brief 实现PublishTogether \n
     * @brief 判断是否找到目标， 找到目标就发送目标数据 \n
     * @brief 否则经过一定时间之后， 将gimbalControlData的GimbalAngles均匀变化
     */
    void Application::PublishTogether() {

        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        static constexpr auto delta_yaw = 1.0f;
        static constexpr auto buff_yaw = -50.0f + 360.0f;

        
        // 小陀螺检查
        const auto navi_speed_vector_length = std::sqrt(
                    std::pow(static_cast<float>(naviVelocity.X), 2) +
                    std::pow(static_cast<float>(naviVelocity.Y), 2)
                    );
        if(healthDecreaseDetector.trigger(myselfHealth)) { // 血量减少
            rotateTimerClock.tick();
            
            if (navi_speed_vector_length > 50)
                gimbalControlData.FireCode.Rotate = 0;
            else if (navi_speed_vector_length > 35)
                gimbalControlData.FireCode.Rotate = 1;
            else if (navi_speed_vector_length > 15)
                gimbalControlData.FireCode.Rotate = 2;
            else
                gimbalControlData.FireCode.Rotate = 3;
        }else {
            if(rotateTimerClock.trigger()){ // 超过两秒没有掉血
                gimbalControlData.FireCode.Rotate = 1;
            }
            if (navi_speed_vector_length > 50)
                gimbalControlData.FireCode.Rotate = 0;
        }
        if (config.AimDebugSettings.StopRotate) gimbalControlData.FireCode.Rotate = 0;

        LoggerPtr->Info("Rotate Speed: {}", gimbalControlData.FireCode.Rotate);


        /*----------云台----------*/
        bool FindTarget = isFindTargetAtomic;
        auto now = std::chrono::steady_clock::now();
        if (FindTarget) {
            LoggerPtr->Info("Find Target");
            if (!config.AimDebugSettings.StopFire){
                if(aimMode != AimMode::Buff) { // 非打符模式，即打前哨和打车
                    if(fireRateClock.trigger()){
                        fireRateClock.tick();
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                }else{ // 打符模式
                    if(buffAimData.FireStatus){
                        /// 立刻响应不需要tick;
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                        buffAimData.FireStatus = false;
                        buff_shoot_count++;
                    }
                }
            }
            gimbalControlData.FireCode.AimMode = 1;
            lastFoundEnemyTime = now;
            isFindTargetAtomic = false;
            
            if(aimMode == AimMode::Buff) {
                gimbalControlData.GimbalAngles = buffAimData.Angles;
            }
            else if(aimMode == AimMode::Outpost) {
                gimbalControlData.GimbalAngles = outpostAimData.Angles;
            } else {
                gimbalControlData.GimbalAngles = autoAimData.Angles;
                LoggerPtr->Info("Pitch: {}, Yaw: {}", autoAimData.Angles.Pitch, autoAimData.Angles.Yaw);
            }
        }
        else { // 未识别到目标
            if(aimMode != AimMode::Buff) {
                /// 云台控制数据均匀变化
                if (!config.AimDebugSettings.StopScan && now - lastFoundEnemyTime > std::chrono::milliseconds(2000)) {
                    gimbalControlData.FireCode.AimMode = 0;
                    LoggerPtr->Info("Searching Target...");
                    const auto current_time = std::chrono::steady_clock::now();
                    gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw + 3 * delta_yaw;
                    // gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw + delta_yaw;
                    gimbalControlData.GimbalAngles.Pitch =
                        AngleType{-0.0f + pitch_wave.Produce(current_time) * 3.0f};

                    if (aimMode == AimMode::Outpost) {
                        gimbalControlData.GimbalAngles.Pitch += 15.0f;
                    }
                } else { // 丢失目标之后，两秒钟只能依旧转发旧角度
                    // if(aimMode != AimMode::Buff){
                        gimbalControlData.GimbalAngles = autoAimData.Angles;
                        LoggerPtr->Info("Pitch: {}, Yaw: {}", autoAimData.Angles.Pitch, autoAimData.Angles.Yaw);
                    // }
                    // else if(aimMode == AimMode::Buff) gimbalControlData.GimbalAngles = buffAimData.Angles;
                }
            }else { // 打符模式
                if (now_time < 5) {
                    LoggerPtr->Info("Set Angles, Buff Mode, 10 min!");
                    // Yaw
                    float current_yaw = normalize_angle_0_360(gimbalAngles.Yaw);
                    float delta = buff_yaw - current_yaw;
                    if (delta > 180.0f) delta -= 360.0f; // 角度差大于180，反向旋转
                    if (delta < -180.0f) delta += 360.0f;
                    int opt = delta > 0 ? 1 : -1;
                    int target_yaw = gimbalAngles.Yaw + delta;

                    if (std::abs(delta) > 10 * delta_yaw) {
                        gimbalControlData.GimbalAngles.Yaw += delta_yaw * opt;
                    } else {
                        gimbalControlData.GimbalAngles.Yaw = target_yaw;
                    }
                    // Pitch
                    gimbalControlData.GimbalAngles.Pitch = 19.0f;

                    gimbalControlData.FireCode.AimMode = 0;
                }else {
                    if(buffAimData.BuffFollow) gimbalControlData.GimbalAngles = buffAimData.Angles;
                }
            }
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
        }
        if(naviLowerHead) {
            gimbalControlData.GimbalAngles.Pitch = -15.0f; //-22.5 - 26.0
        }
        

        PublishMessageAll();
    }

    /**
     * @brief 决策进程主循环 \n
     * @brief 1. 设置黑板数据 \n
     * @brief 2. 休眠 \n
     * @brief 3. 更新黑板数据 \n
     * @brief 4. 处理行为树 \n
     */
    void Application::GameLoop() {

        BlackBoard->set<UnitTeam>("MyTeam", team); // 队伍颜色
        BlackBoard->set<std::chrono::steady_clock::time_point>(
            "LastCommandTime", std::chrono::steady_clock::now()); // 上次发送命令的时间
        BlackBoard->set<std::chrono::seconds>("CommandInterval", std::chrono::seconds{0}); // 命令间隔
        BlackBoard->set<ArmorType>("AimTarget", ArmorType::Hero); // 辅瞄击打目标
        BlackBoard->set<std::uint8_t>("naviCommandGoal", Home(team)); // 导航目的地
        BlackBoard->set<std::shared_ptr<Logger>>("LoggerPtr", LoggerPtr);

        while (ros::ok()) {
            TransportData();
            treeTickRateClock.sleep();
            ros::spinOnce(); // 处理回调函数
            UpdateBlackBoard();
            LoggerPtr->Info("--------------------> BehaviorTree Root Tick Done! <--------------------");
            LoggerPtr->Info("--------------------> BehaviorTree Root Tick! <--------------------");
            TreeTick();
        }
    }

    void Application::TreeTick() {
        SetAimMode(); // 处理辅瞄模式
        CheckDebug(); // 处理调试状态
        ProcessData(); // 处理机器人数据
        SetPositionRepeat(); // 设置位置
        SetAimTarget(); // 设置击打目标
        // BTree.tickRoot();
    }

    void Application::SetAimMode() {
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        LoggerPtr->Info("SetAimMode - now_time: {}", now_time);
        if(config.GameStrategySettings.HitBuff) { // 打符
            if(now_time < 25 && buff_shoot_count <= 15){
                if(now_time > 7) {
                    if(teamBuff.DefenceBuff > 20 || teamBuff.VulnerabilityBuff > 20){
                        LoggerPtr->Info("Buff has been activated!");
                        LoggerPtr->Info("Defense Buff: {}, Vulnerability Buff: {}", teamBuff.DefenceBuff, teamBuff.VulnerabilityBuff);
                        aimMode = AimMode::RotateScan;
                    }
                    else{
                        LoggerPtr->Info("Buff has not been activated!");
                        aimMode = AimMode::Buff;
                    }
                }else aimMode = AimMode::Buff;
                
            }else {
                LoggerPtr->Info("Time out 25 seconds, stop hit buff!");
                aimMode = AimMode::RotateScan;
            }
        }else if(config.GameStrategySettings.HitOutpost) { // 打前哨站
            if(enemyOutpostHealth > 0) {
                LoggerPtr->Info("Enemy Outpost Health: {}", enemyOutpostHealth);;
                if(now_time < 90) {
                    aimMode = AimMode::Outpost;
                }else {
                    LoggerPtr->Info("Time out 1.5 min, stop hit outpost!");
                    aimMode = AimMode::RotateScan;
                }
            }else {
                LoggerPtr->Info("Enemy Outpost has been destroyed!");
                aimMode = AimMode::RotateScan;
            }
        }else { // 普通模式
            LoggerPtr->Info("AimMode: RotateScan!");
            aimMode = AimMode::RotateScan;
        }

    }
    // 提前处理坐标等数据
    void Application::ProcessData() {
        int now_time = 420 - timeLeft;
        // 处理坐标数据
        reliableEnemyPosuition.clear();
        for(auto robot : RobotLists) {
           if(enemyRobots[robot].position_.X > 100 && enemyRobots[robot].position_.Y > 100) {
                reliableEnemyPosuition.push_back(robot);
            }
        }
        LoggerPtr->Info("> reliableEnemyPosuition <");
        for(auto robot : reliableEnemyPosuition) {
            LoggerPtr->Info("ID: {}, X: {}, Y:{}", static_cast<int>(robot), enemyRobots[robot].position_.X, enemyRobots[robot].position_.Y);
        }

        // 处理距离和无敌状态的数据
        hitableTargets.clear();
        for (auto Armor : armorList) {
            if (Armor.Type == ArmorType::UnKnown) continue;
            if(Armor.Type == ArmorType::Hero) {
                enemyRobots[UnitType::Hero].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Hero].isInvulnerable()) hitableTargets.push_back(UnitType::Hero);
            }else if(Armor.Type == ArmorType::Engineer) {
                enemyRobots[UnitType::Engineer].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Engineer].isInvulnerable() && now_time > 60) hitableTargets.push_back(UnitType::Engineer);
            }else if(Armor.Type == ArmorType::Infantry1) {
                enemyRobots[UnitType::Infantry1].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry1].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry1);
            }else if(Armor.Type == ArmorType::Infantry2) {
                enemyRobots[UnitType::Infantry2].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Infantry2].isInvulnerable()) hitableTargets.push_back(UnitType::Infantry2);
            }else if(Armor.Type == ArmorType::Sentry) {
                enemyRobots[UnitType::Sentry].distance_ = Armor.Distance;
                if(!enemyRobots[UnitType::Sentry].isInvulnerable()) hitableTargets.push_back(UnitType::Sentry);
            }
        }
        for(auto robot : hitableTargets) {
            LoggerPtr->Info("ID{}", static_cast<int>(robot));
        }
    }

    void Application::SetAimTarget() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        std::uint16_t nowx = friendRobots[UnitType::Sentry].position_.X, nowy = friendRobots[UnitType::Sentry].position_.Y;
        if(aimMode == AimMode::Buff) { // 打符，修改为默认值
            if(BehaviorTree::Area::BuffShoot.near(nowx, nowy, 100, MyTeam)) {
                targetArmor.Type = ArmorType::Hero;
            } else {
                SetAimTargetNormal();
            }
        }else if(aimMode == AimMode::Outpost) { // 打前哨站
            if(BehaviorTree::Area::OutpostShoot.near(nowx, nowy, 100, MyTeam)) {
                targetArmor.Type = ArmorType::Outpost;
            } else {
                SetAimTargetNormal();
            }
        }else { // 普通模式
            if(naviCommandGoal == LangYa::HoleRoad(EnemyTeam)) { // 英雄点位1
                if(BehaviorTree::Area::HoleRoad.near(nowx, nowy, 100, MyTeam)) {
                    targetArmor.Type = ArmorType::Hero;
                } else {
                    SetAimTargetNormal();
                }
            }
            LoggerPtr->Info("Target: {}", static_cast<int>(targetArmor.Type));
        }

    }

    void Application::SetAimTargetNormal() {
        if(hitableTargets.size() > 0) {
            bool hero_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Hero) != hitableTargets.end());
            bool sentry_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Sentry) != hitableTargets.end());
            bool engnieer_find = (std::find(hitableTargets.begin(), hitableTargets.end(), UnitType::Engineer) != hitableTargets.end());
            bool infantry1_find = (std::find(hitableTargets.begin(),  hitableTargets.end(), UnitType::Infantry1) != hitableTargets.end());
            bool infantry2_find = (std::find(hitableTargets.begin(),  hitableTargets.end(), UnitType::Infantry2) != hitableTargets.end());
            if (hero_find) {
                targetArmor.Type = ArmorType::Hero;
                targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
            }else if(infantry1_find || infantry2_find) {
                if(infantry1_find && !infantry2_find) {
                    targetArmor.Type = ArmorType::Infantry1;
                    targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                }else if(!infantry1_find && infantry2_find) {
                    targetArmor.Type = ArmorType::Infantry2;
                    targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
                }else {
                    int delta_distance = enemyRobots[UnitType::Infantry1].distance_ - enemyRobots[UnitType::Infantry2].distance_;
                    int delta_health = enemyRobots[UnitType::Infantry1].currentHealth_ - enemyRobots[UnitType::Infantry2].currentHealth_;
                    if(std::fabs(delta_distance) > 1) {
                        if(enemyRobots[UnitType::Infantry1].distance_ < enemyRobots[UnitType::Infantry2].distance_) {
                            targetArmor.Type = ArmorType::Infantry1;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                        }else {
                            targetArmor.Type = ArmorType::Infantry2;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry2].
                        }
                    } else { 
                        if(delta_health < 0) {
                            targetArmor.Type = ArmorType::Infantry1;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
                        }else {
                            targetArmor.Type = ArmorType::Infantry2;
                            targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
                        }
                    }
                } 
            }else if(sentry_find){
                targetArmor.Type = ArmorType::Sentry;
                targetArmor.Distance = enemyRobots[UnitType::Sentry].distance_;
            }else if(engnieer_find) {
                targetArmor.Type = ArmorType::Engineer;
                targetArmor.Distance = enemyRobots[UnitType::Engineer].distance_;
            }
            // else {
            //     int min_health = 9999;
            //     UnitType min_health_unit;
            //     for(auto unit : hitableTargets) {
            //         if(enemyRobots[unit].currentHealth_ < min_health) {
            //             min_health = enemyRobots[unit].currentHealth_;
            //             min_health_unit = unit;
            //         }
            //     }
            //     if(min_health_unit == UnitType::Engineer) {
            //         targetArmor.Type = ArmorType::Engineer;
            //         targetArmor.Distance = enemyRobots[UnitType::Engineer].distance_;
            //     }else if(min_health_unit == UnitType::Infantry1) {
            //         targetArmor.Type = ArmorType::Infantry1;
            //         targetArmor.Distance = enemyRobots[UnitType::Infantry1].distance_;
            //     }else if(min_health_unit == UnitType::Infantry2) { 
            //         targetArmor.Type = ArmorType::Infantry2;
            //         targetArmor.Distance = enemyRobots[UnitType::Infantry2].distance_;
            //     }else if(min_health_unit == UnitType::Sentry) {
            //         targetArmor.Type = ArmorType::Sentry;
            //         targetArmor.Distance = enemyRobots[UnitType::Sentry].distance_;
            //     }else if(min_health_unit == UnitType::Hero) {
            //         targetArmor.Type = ArmorType::Hero;
            //         targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
            //     }
            // }
        }else {
            targetArmor.Type = ArmorType::Hero;
            targetArmor.Distance = 30;
        }
    }

    void Application::CheckDebug() {
        if (config.AimDebugSettings.HitBuff) aimMode = AimMode::Buff;
        else if(config.AimDebugSettings.HitOutpost) aimMode = AimMode::Outpost; 
        /*------------打印日志---------*/
        if(aimMode == AimMode::AutoAim) LoggerPtr->Info("AimMode: AutoAim");
        else if(aimMode == AimMode::Buff) LoggerPtr->Info("AimMode: Buff");
        else if(aimMode == AimMode::Outpost) LoggerPtr->Info("AimMode: Outpost");
        else if(aimMode == AimMode::RotateScan) LoggerPtr->Info("AimMode: RotateScan");
    }
}


namespace BehaviorTree {

    void Application::SetPositionRepeat() {
        if(config.GameStrategySettings.HitSentry) SetPositionHitSentry();
        else if(config.GameStrategySettings.TestNavi) SetPositionNaviTest();
        else if(config.GameStrategySettings.Protected) SetPositionProtect();
        else SetPositionHitHero();
    }

    bool Application::CheckPositionRecovery() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        // 复活
        if(naviCommandGoal == Recovery(MyTeam)) {
            if(myselfHealth < 380) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }
        }
        // 回家
        // 条件为：血量低于150 或者 弹药为0且距离上一次回家已经过去90秒
        if(myselfHealth < 100 || (ammoLeft <= 30 && recoveryClock.trigger())) {
            SET_POSITION(Recovery, MyTeam);
            recoveryClock.tick();
            naviCommandIntervalClock.reset(Seconds{1});
            return true;
        }
        return false;
    }

    void Application::SetPositionProtect() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        
        // 检测是否需要回家
        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }

        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }        
        if(aimMode == AimMode::Buff) { 
            SET_POSITION(BuffShoot, MyTeam); // 打符
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2; //加速
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1; // 正常
        }else { //普通模式
            Random random;
            
                int random_number = random.Get(0, 6);
                if (random_number == 0) SET_POSITION(CastleLeft, MyTeam);
                else if (random_number == 1) SET_POSITION(CastleRight1, MyTeam);
                else if (random_number == 2) SET_POSITION(CastleRight2, MyTeam);
                else SET_POSITION(BuffShoot, MyTeam);
                if(naviCommandGoal == BuffShoot(MyTeam)) naviCommandIntervalClock.reset(Seconds{30});
                else naviCommandIntervalClock.reset(Seconds(10));
            // 时间超过5分钟 或 底盘能量低于5%
            if(now_time > 300 || 
                 
                teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                // naviCommandGoal = Castle(MyTeam);
                speedLevel = 0;
            } //else speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }

    void Application::SetPositionNaviTest() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        
        if(now_time < 20) SET_POSITION(BuffShoot, MyTeam);
        else if(now_time < 40) SET_POSITION(LeftHighLand, MyTeam);
        else if(now_time < 60) SET_POSITION(CastleLeft, MyTeam);
        else if(now_time < 80) SET_POSITION(CastleRight1, MyTeam);
        else if(now_time < 100) SET_POSITION(CastleRight2, MyTeam);
        else if(now_time < 120) SET_POSITION(FlyRoad, MyTeam);
        else if(now_time < 140) SET_POSITION(OutpostArea, MyTeam);
        else if(now_time < 160) SET_POSITION(MidShoot, MyTeam);
        else if(now_time < 180) SET_POSITION(LeftShoot, MyTeam);
        else if(now_time < 200) SET_POSITION(OutpostShoot, MyTeam);
        else if(now_time < 220) SET_POSITION(FlyRoad, EnemyTeam);
        else if(now_time < 240) SET_POSITION(CastleRight1, EnemyTeam);
        else if(now_time < 260) SET_POSITION(CastleRight2, EnemyTeam);
        else if(now_time < 280) SET_POSITION(CastleLeft, EnemyTeam);
        else if(now_time < 300) SET_POSITION(LeftHighLand, EnemyTeam);
        else if(now_time < 320) SET_POSITION(BuffShoot, EnemyTeam);
        else if(now_time < 340) SET_POSITION(OutpostArea, EnemyTeam);
        else SET_POSITION(Castle, MyTeam);
    }

    void Application::SetPositionHitSentry() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        if(aimMode == AimMode::Buff) { 
            SET_POSITION(BuffShoot, MyTeam); // 打符
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam); // 打前哨站
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            Random random;

                // if(!hitableTargets.empty()) { // 视野里存在目标
                //     bool low_health_enemy = false;
                //     // 检测低血量敌人
                //     for(auto robot : hitableTargets) {
                //         if(enemyRobots[robot].currentHealth_ < 50) {
                //             low_health_enemy = true;
                //         }
                //     }
                bool infantry1_in_central = 
                    Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y)
                    || Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y);
                bool infantry2_in_central = 
                    Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y)
                    || Area::CentralHighLandBlue.isPointInside(enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y);

                if(infantry1_in_central) {
                    LoggerPtr->Debug("Infantry1 in Highland");
                }
                if(infantry2_in_central) {
                    LoggerPtr->Debug("Infantry2 in Highland");
                }
                


                    if(selfOutpostHealth > 100 && now_time < 55 ) {
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }else {
                        SET_POSITION(FlyRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(10));
                    }
                    
             // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                SET_POSITION(HoleRoad, MyTeam);
                naviCommandIntervalClock.reset(Seconds(10));
                speedLevel = 0;
            } // else speedLevel = 1;
            speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }

    void Application::SetPositionHitHero() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();

        if(CheckPositionRecovery()) {
            // if(teamBuff.RemainingEnergy) speedLevel = 1;
            // else speedLevel = 2;
            LoggerPtr->Info("Health: {}, AmmoLeft: {}", myselfHealth, ammoLeft);
            LoggerPtr->Info("> Go Recovery");
            return;
        }
        // 防止高速切换指令
        if(!naviCommandIntervalClock.trigger()) {
            return;
        }
        if(aimMode == AimMode::Buff) { 
            SET_POSITION(BuffShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 2;
        }else if (aimMode == AimMode::Outpost) {
            SET_POSITION(OutpostShoot, MyTeam);
            naviCommandIntervalClock.reset(Seconds(2));
            // speedLevel = 1;
        }else { //普通模式
            Random random;
            // 判断英雄是否处于高地
            bool hero_in_central = false;
            std::int16_t hero_x = enemyRobots[UnitType::Hero].position_.X, hero_y = enemyRobots[UnitType::Hero].position_.Y;
            hero_in_central = Area::CentralHighLandRed.isPointInside(hero_x, hero_y) || Area::CentralHighLandBlue.isPointInside(hero_x, hero_y);

            if(hero_in_central) {
                LoggerPtr->Debug("!!!Hero in highland!!!");
                SET_POSITION(BuffAround1, MyTeam);
            }else {
                if(selfOutpostHealth > 200) {
                    int redpx = 982, redpy = 1124;
                    int dx = redpx - enemyRobots[UnitType::Hero].position_.X, dy = redpy - enemyRobots[UnitType::Hero].position_.Y;
                    int len = std::sqrt(dx * dx + dy * dy);
                    if(len < 100) {
                        SET_POSITION(HoleRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(2));
                    }else {
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }
                }else {
                    if(selfBaseHealth > 2000){
                        SET_POSITION(HoleRoad, EnemyTeam);
                        naviCommandIntervalClock.reset(Seconds(2));
                    }
                    else {
                        int random_number = random.Get(0, 8);
                        if(random_number == 0) SET_POSITION(MidShoot, MyTeam);
                        else if(random_number == 1) SET_POSITION(BuffAround1, MyTeam);
                        else if(random_number == 2) SET_POSITION(BuffAround2, MyTeam);
                        else if(random_number == 3) SET_POSITION(RightShoot, MyTeam);
                        else if(random_number == 4) SET_POSITION(MidShoot, EnemyTeam);
                        else if(random_number == 5) SET_POSITION(BuffAround1, EnemyTeam);
                        else if(random_number == 6) SET_POSITION(BuffAround2, EnemyTeam);
                        else if(random_number == 7) SET_POSITION(RightShoot, EnemyTeam);
                        else if(random_number == 8) SET_POSITION(LeftShoot, EnemyTeam);
                        if(naviCommandGoal == MidShoot(EnemyTeam) || naviCommandGoal == LeftShoot(EnemyTeam))
                            naviCommandIntervalClock.reset(Seconds(8));
                        else naviCommandIntervalClock.reset(Seconds(10));
                    }
                }
            }
            
            // 底盘能量低于5%
            if(teamBuff.RemainingEnergy == 0b10000 || teamBuff.RemainingEnergy == 0b00000) {
                LoggerPtr->Info("!!! Low Energy !!!");
                int random_number = random.Get(0, 2);
                if(random_number == 0) SET_POSITION(BuffAround1, MyTeam);
                else if(random_number == 1) SET_POSITION(BuffAround2, MyTeam);
                else if(random_number == 2) SET_POSITION(RightShoot, MyTeam);
                naviCommandIntervalClock.reset(Seconds(10));
                speedLevel = 0;
            } // else speedLevel = 1;
            speedLevel = 1;
            LoggerPtr->Info("> NaviCommandGoal: {}", static_cast<int>(naviCommandGoal));
        }
    }



}