def get_motion_cmd(linear_velocity:float, angular_velocity: float):
    """
    Twist型のメッセージから動作に関するコマンドを取得する

    Parameters
    ----------
    linear_velocity: float
        並進速度
    angular_velocity: float
        旋回速度

    Returns
    -------
    cmd : str
        送信するコマンド
    """

    cmd = 'S\n'

    # 並進速度＞０で前進するコマンド
    if(linear_velocity > 0.0):
        cmd = 'F\n'

    # 並進速度＜０で後退するコマンド
    elif(linear_velocity < 0.0):
        cmd = 'B\n'

    # 旋回速度＞０で右旋回
    elif(angular_velocity > 0.0):
        cmd = 'R\n'

    # 旋回速度＜０で左旋回
    elif(angular_velocity < 0.0):
        cmd = 'L\n'

    # 並進速度も旋回速度も入力がない場合、ストップするコマンド
    # elif(msg.angular.z == 0.0 & msg.linear.y == 0.0):
    #     send_cmd = 'S\n'

    return cmd

def get_velocity_cmd(vel:float):
    """
    floatの速度(0.0から1.0)より車輪速度(1から9)を取得する。

    vel > 1.0の場合は9が出力。

    vel < 0.0の場合は絶対値で計算。

    Parameters
    ----------
    vel: float
        速度
    
    Returns
    -------
    cmd : int
        
    """

    # １０倍することで0.1~1.0の数字なら1~10の数字になる
    fixed_vel = 10 * vel

    if(fixed_vel < 0.0):
        fixed_vel = fixed_vel * -1.0

    # int型にキャストすることで小数点以下切り捨て
    cmd = (int)(fixed_vel)

    # １より小さい場合１にする
    if(cmd < 1):
        cmd = 1
    # ９より大きいなら９にする
    elif(cmd > 9):
        cmd = 9
    

    return cmd