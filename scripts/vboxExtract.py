import xlrd
import numpy as np
import pandas as pd

open_XLSX = "/media/yoyo/harddisk/ncisit/VBOX0182_第一段_update.xlsx"
save_CSV  = "/media/yoyo/harddisk/ncisit/vbox-01.csv"


pd.set_option('display.float_format', lambda x: '%.3f' % x)

def Col2Num(col):
    return sum((ord(c) - 64) * 26**i for i, c in enumerate(reversed(col))) - 1


workbook = xlrd.open_workbook(open_XLSX)
booksheet: xlrd.sheet.Sheet = workbook.sheet_by_index(0)

data = []
data += [booksheet.col_values(Col2Num('A'))[1:]]
data += [[i.replace('簞', '.').replace(' N', '') for i in (booksheet.col_values(Col2Num('E'))[1:])]]
data += [[i.replace('簞', '.').replace(' E', '') for i in (booksheet.col_values(Col2Num('F'))[1:])]]
data += [booksheet.col_values(Col2Num('M'))[1:]]
data += [booksheet.col_values(Col2Num('D'))[1:]]
data += [booksheet.col_values(Col2Num('C'))[1:]]
data += [booksheet.col_values(Col2Num('AR'))[1:]]
data += [booksheet.col_values(Col2Num('AS'))[1:]]
data += [booksheet.col_values(Col2Num('AT'))[1:]]

groups = ['UTC time',               # A  0  UTCTime
          'Latitude(deg.min) N',    # F  5  緯度　　　Latitude
          'Longitude(deg.min) E',   # E  4  經度　　　Longitude
          'Height (m)',             # M  12 高度　　　Height
          'Heading (Degrees)',      # D  3  角度　　　Heading
          'Speed (km/h)',           # C  2  速度　　　Speed
          'YawRate (deg/s)',        # AR 43 角速度　　yaw-rate
          'X_Accel (g)',            # AS 44 Ｘ加速度　X_Accel
          'Y_Accel (g)'             # AT 45 Ｙ加速度　Y_Accel
         ]

n_data = np.array(data)
n_data = n_data.transpose()

df = pd.DataFrame(n_data)
df.columns = groups
df.to_csv(save_CSV, index=False)
