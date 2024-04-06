#メイン処理
#使用する前に全ての関数について調べどのような役割があるのか確認すること
def main() :
    #セットアップ
    log_setup()                                   
    moter_setup()                                 
    MPU_setup()                                  
    BME_setup()                                  
    get_calib_param()                            
    goal_point = get_gps_goal()
    preset_pres = get_preset_pres()                  
    sequence_count = 0                           
    sequence_count = get_sequence_count()        
    write_log(sequence_count,diredtion="set_up") 
    print("set_up_OK")                    

    #投下シーケンス
    #フォトレジスタの電圧が2.0V以上になった(CANSATが投下された)ならば無限ループを抜ける処理が行われる
    if sequence_count == 0 :
        
        while True :
            photoresistor_voltage = float(read_voltage()) 
            
            if photoresistor_voltage > 2.0 :
                sequence_count = 1 
                write_sequence_count(sequence_count) 
                #write_im920flag(1) 
                print("release")
                break
            
            else :
                time.sleep(1)  
    
    #第二投下シーケンス(光センサーの使用が不可の場合に使用)
    #上昇と降下を検知して投下判定を行う
    if sequence_count == 0.5 :
        altitude_flag = 0 
        while True :
            if get_altitude(preset_pres) > preset_altiude2 :
                print("rise")
                
                if altitude_flag < 3 :
                    altitude_flag = altitude_flag + 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise2" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                
                elif altitude_flag == 3 :
                    sequence_count = 1
                    write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                              diredtion = "rise3" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                              goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
                    break
            
            elif get_altitude < preset_altiude2 :
                write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                          diredtion = "rise1" ,my_angle = 0 ,goal_angle = get_goal_angle(get_gps_realtime(),goal_point) ,
                          goal_destance = get_goal_distance(get_gps_realtime(),goal_point))
            
            time.sleep(0.1)
    
    #降下シーケンス
    if sequence_count == 1 :
        time_flag = 1
        while True :
            
            if get_altitude(preset_pres) < preset_altiude :
                time.sleep(10)
                print("landing_1")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                break
            
            #競技によって予想着地時間が違うので競技ごとにパラ分離までの時間を設定してください
            elif time_flag > 600 :
                time.sleep(5)
                print("landing_2")
                career_cat(career_time)
                sequence_count = 2
                write_sequence_count(sequence_count)
                break
            
            else :
                time.sleep(1.0)
                time_flag = time_flag + 1
                print(time_flag)
    
    #脱出シーケンス
    if sequence_count == 2 :  
        #回収モジュールから脱出
        escape()
        career_cat(career_time)
        sequence_count = 3
        my_point  = get_gps_realtime()
        write_sequence_count(sequence_count)
        write_log(sequence_count,gps = get_gps() ,altitude = get_altitude(preset_pres) ,photvolt = 0 ,
                  diredtion = "escape" ,my_angle = 0 ,goal_angle = get_goal_angle(my_point,goal_point) ,
                  goal_destance = get_goal_distance(my_point,goal_point))
        print("脱出")