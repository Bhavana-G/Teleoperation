import pandas as pd

orig_df = pd.read_excel('./Imp1.xlsx', skiprows=7)
df = orig_df[['Quat_W', 'Quat_X', 'Quat_Y', 'Quat_Z']]
print(orig_df.head(10))
print(df)