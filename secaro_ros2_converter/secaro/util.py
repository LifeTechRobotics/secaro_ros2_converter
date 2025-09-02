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

    # どの場合にも入らない場合には停止してほしいので停止コマンドで初期化
    cmd = 'S\n'

    # 「並進速度 > 旋回速度」で並進扱いにする
    if(abs(linear_velocity) > abs(angular_velocity)):
        # 「並進速度 > 0」で前進
        if(linear_velocity > 0.0):
            cmd = 'F\n'
        # 「並進速度 > 0」で後退
        else:
            cmd = 'B\n'
    # 「並進速度 < 旋回速度」で旋回扱いにする
    elif(abs(linear_velocity) < abs(angular_velocity)):
        # 「旋回速度 > 0」で左旋回
        if(angular_velocity > 0.0):
            cmd = 'L\n'
        # 「旋回速度 < 0」で右旋回
        else:
            cmd = 'R\n'
    elif(linear_velocity == 0.0 & angular_velocity == 0.0):
        # 「入力なし」で停止
        cmd = 'S\n'

    return cmd

def get_velocity_cmd(linear_velocity:float, angular_velocity: float, wheel_base: float):
    """
    floatの速度(0.0から1.0)より車輪速度(1から9)を取得する。

    vel > 1.0の場合は9が出力。

    vel < 0.0の場合は絶対値で計算。

    Parameters
    ----------
    linear_velocity: float
        並進速度
    angular_velocity: float
        旋回速度
    wheel_base: float
        左右の車輪間の距離[m]
    
    Returns
    -------
    (left_cmd, right_cmd) : (int,int)
        
    """


    # １０倍することで0.1~1.0の数字なら1~10の数字になる
    left_fixed_vel = 10 * (linear_velocity - angular_velocity * wheel_base / 2.0)
    right_fixed_vel = 10 * (linear_velocity + angular_velocity * wheel_base / 2.0)

    if(left_fixed_vel < 0.0):
        left_fixed_vel = left_fixed_vel * -1.0

    if(right_fixed_vel < 0.0):
        right_fixed_vel = right_fixed_vel * -1.0

    # int型にキャストすることで小数点以下切り捨て
    left_cmd = (int)(left_fixed_vel)
    right_cmd = (int)(right_fixed_vel)

    left_cmd = check_over(left_cmd, 1, 9)
    right_cmd = check_over(right_cmd, 1, 9)

    return (left_cmd, right_cmd)

def check_over(target, min_value, max_value):
    """
    値が最大最小の範囲から出ていないか調べる

    Parameters
    ----------
    target
        対象の値
    min_value
        最小の値
    max_value
        最大の値
    
    Returns
    -------
    target
        
    """
    if(target > max_value):
        return max_value
    elif(target < min_value):
        return min_value
    else:
        return target