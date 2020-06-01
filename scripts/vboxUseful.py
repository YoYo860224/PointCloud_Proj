import numpy as np
import pandas as pd
import twd97

rd_CSV = "/media/yoyo/harddisk/ncisit/vbox-01.csv"
wr_CSV = "/media/yoyo/harddisk/ncisit/u_vbox-01.csv"


pd.set_option('display.float_format', lambda x: '%.3f' % x)

def wgs_unit_D(str_DM: str):
    dotID = str_DM.find('.')
    return float(str_DM[:dotID]) + float(str_DM[dotID+1:]) / 60.0


groups = ['UTC time',               # UTCTime
          'X',                      # TWD97 橫軸 (m)
          'Y',                      # TWD97 縱軸 (m)
          'Z',                      # 高度 (m)
          'Heading',                # 角度 (Degrees)
          'Speed',                  # 速度 (m/s)
          'YawRate',                # 角速度 (deg/s)
          'X_Accel',                # Ｘ加速度 (m/s^2)
          'Y_Accel'                 # Ｙ加速度 (m/s^2)
         ]

df = pd.read_csv(rd_CSV)
df.columns = groups
df.iloc[:, 1] = [wgs_unit_D(i) for i in df.iloc[:, 1]]
df.iloc[:, 2] = [wgs_unit_D(i) for i in df.iloc[:, 2]]

for i in df.axes[0]:
    twd97Pos = twd97.fromwgs84(df.iloc[i, 1], df.iloc[i, 2])
    df.iloc[i, 1] = twd97Pos[0]
    df.iloc[i, 2] = twd97Pos[1]
    df.iloc[i, 5] = (df.iloc[i, 5] * 1000.0 / 3600.0)
    df.iloc[i, 7] = (df.iloc[i, 7] * 9.8)
    df.iloc[i, 8] = (df.iloc[i, 8] * 9.8)

df.to_csv(wr_CSV, index=False)
