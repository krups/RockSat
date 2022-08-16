from pyparsing import col
import serial
import matplotlib.pyplot as plt
import numpy as np
import sys
import pandas as pd
from time import sleep
from os import listdir
from os.path import isfile, join, isdir
import plotly
import plotly.express as px
import plotly.graph_objects as go
from datetime import datetime

current_time = datetime.now().strftime("%m-%d_%H%M")
csv_save_path = f"csv_bin/parsed_data_{current_time}.csv"

################################
#
#       build dataframe
#
################################

####################
#   Preprocessing
####################
col_names = ['Type', 'Time', 'Val_1', 'Val_2',
             'Val_3', 'Val_4', 'Val_5', 'Val_6']

# Import dataframe from existing file
if (len(sys.argv) > 1):
    if isfile(sys.argv[1]):
        print(f"Importing data from {sys.argv[1]}")
        df = pd.read_csv(sys.argv[1])
        print(df)
    else:
        print(f"Error: File \"{sys.argv[1]}\" does not exist.")
        exit()

# Create dataframe from parser output
else:
    parser_outputs_dir = "./parser_output/"
    if not isdir(parser_outputs_dir):
        print("Error: Parser output folder doesn't exist")
        exit()
    parser_outputs = [join(parser_outputs_dir, f) for f in listdir(
        parser_outputs_dir) if isfile(join(parser_outputs_dir, f))]
    df = pd.DataFrame(columns=col_names)

    for input in parser_outputs:
        IN = open(input, "r").read()
        IN = IN.split('\n')
        lines = IN[4:-1]
        data = []
        for x in lines:
            temp = x.split(',')
            temp = [y.strip() for y in temp]
            q = [y.split() for y in temp]
            temp[0] = q[0][1]
            temp.insert(0, q[0][0].replace(":", ""))
            while len(temp) != 8:
                temp.append("NaN")
            data.append(temp)
        # append dataframe
        for x in data:
            # print(x)
            temp_df = pd.DataFrame([x], columns=col_names)
            df = pd.concat([df, temp_df], ignore_index=True)
    df = df.apply(pd.to_numeric, errors='ignore')
    print("Dataframe created from parser output")
    ####################
    #     Sort CSV
    ####################
    df = df.sort_values(by=col_names[1], ignore_index=True)
    print(f"Dataframe sorted by {col_names[1]}")

    ####################
    #     Save CSV
    ####################
    print(f"Dataframe saved to {csv_save_path}")
    df.to_csv(csv_save_path, index_label=False, index=False)
    print(df)

################################
#
#       Display Figures
#
################################
plt.style.use('ggplot')

########################
#   Split data by type
########################
IMU_df = pd.DataFrame(columns=col_names)
TC_df = pd.DataFrame(columns=col_names)
GGA_df = pd.DataFrame(columns=col_names)
GGA_df_1 = pd.DataFrame(columns=col_names)
RMC_df = pd.DataFrame(columns=col_names)
RMC_df_1 = pd.DataFrame(columns=col_names)
ACC_df = pd.DataFrame(columns=col_names)
SPEC_df = pd.DataFrame(columns=col_names)
for i in range(len(df)):
    type = df.iloc[i]["Type"]
    temp_df = pd.DataFrame([df.iloc[i]], columns=col_names)
    if type == "IMU":
        IMU_df = pd.concat([IMU_df, temp_df], ignore_index=True)
    if type == "TC":
        TC_df = pd.concat([TC_df, temp_df], ignore_index=True)
    if type == "GGA":
        GGA_df_1 = pd.concat([GGA_df, temp_df], ignore_index=True)
        if not (df.iloc[i]["Val_2"] == "nan") and not (df.iloc[i]["Val_3"] == "nan"):
            GGA_df = pd.concat([GGA_df, temp_df], ignore_index=True)
    if type == "RMC":
        RMC_df_1 = pd.concat([RMC_df, temp_df])
        if not (df.iloc[i]["Val_2"] == "nan") and not (df.iloc[i]["Val_3"] == "nan"):
            RMC_df = pd.concat([RMC_df, temp_df])
    if type == "ACC":
        ACC_df = pd.concat([ACC_df, temp_df])
    if type == "SPEC":
        SPEC_df = pd.concat([SPEC_df, temp_df])
IMU_df = IMU_df.apply(pd.to_numeric, errors='ignore')
TC_df = TC_df.apply(pd.to_numeric, errors='ignore')
GGA_df = GGA_df.apply(pd.to_numeric, errors='ignore')
GGA_df_1 = GGA_df_1.apply(pd.to_numeric, errors='ignore')
RMC_df = RMC_df.apply(pd.to_numeric, errors='ignore')
RMC_df_1 = RMC_df_1.apply(pd.to_numeric, errors='ignore')
ACC_df = ACC_df.apply(pd.to_numeric, errors='ignore')
SPEC_df = SPEC_df.apply(pd.to_numeric, errors='ignore')

print("Opening figures")
####################
#     IMU Plot
####################
IMU_acc_x = IMU_df["Val_1"]
IMU_acc_y = IMU_df["Val_2"]
IMU_acc_z = IMU_df["Val_3"]
IMU_gyr_x = IMU_df["Val_4"]
IMU_gyr_y = IMU_df["Val_5"]
IMU_gyr_z = IMU_df["Val_6"]
IMU_time = IMU_df["Time"]

IMU_axs = []
IMU_fig, IMU_axs = plt.subplots(2, 3, sharex=True)

IMU_axs[0, 0].plot(IMU_time, IMU_acc_x)
IMU_axs[0, 0].set_title("Acc x")

IMU_axs[0, 1].plot(IMU_time, IMU_acc_y, color="blue")
IMU_axs[0, 1].sharey(IMU_axs[0, 0])
IMU_axs[0, 1].set_title("Acc y")

IMU_axs[0, 2].plot(IMU_time, IMU_acc_z, color="green")
IMU_axs[0, 2].sharey(IMU_axs[0, 0])
IMU_axs[0, 2].set_title("Acc z")

IMU_axs[1, 0].plot(IMU_time, IMU_gyr_x, color="purple")
IMU_axs[1, 0].set_title("Gyro x")

IMU_axs[1, 1].plot(IMU_time, IMU_gyr_y, color="#8B8000")
IMU_axs[1, 1].sharey(IMU_axs[1, 0])
IMU_axs[1, 1].set_title("Gyro y")

IMU_axs[1, 2].plot(IMU_time, IMU_gyr_z, color="#ADD8E6")
IMU_axs[1, 2].sharey(IMU_axs[1, 0])
IMU_axs[1, 2].set_title("Gyro z")

IMU_axs[1, 0].set_xlabel("Time (s)")
IMU_axs[1, 1].set_xlabel("Time (s)")
IMU_axs[1, 2].set_xlabel("Time (s)")

IMU_fig.suptitle("IMU Plots")
IMU_fig.tight_layout()

####################
#      TC Plot
####################
TC_1 = TC_df["Val_1"]
TC_2 = TC_df["Val_2"]
TC_3 = TC_df["Val_3"]
TC_4 = TC_df["Val_4"]
TC_5 = TC_df["Val_5"]
TC_6 = TC_df["Val_6"]
TC_time = TC_df["Time"]

TC_fig, TC_axs = plt.subplots()
TC_axs.plot(TC_time, TC_1, label="TC 1", marker="o")
TC_axs.plot(TC_time, TC_2, label="TC 2", marker="^")
TC_axs.plot(TC_time, TC_3, label="TC 3", marker="v")
TC_axs.plot(TC_time, TC_4, label="TC 4", marker="<")
TC_axs.plot(TC_time, TC_5, label="TC 5", marker=">")
TC_axs.plot(TC_time, TC_6, label="TC 6", marker="x")
TC_axs.legend()
TC_axs.set_xlabel("Time (s)")
TC_axs.set_ylabel("Temperature (C)")
TC_fig.suptitle("TC Plots")

####################
#     GGA Plot
####################
px.set_mapbox_access_token(
    "pk.eyJ1IjoiZm9sa2lzaGFjb3JuIiwiYSI6ImNsM2xxNWY3dTAzdDkzY284Z2dnbWZlYm0ifQ.gtXNkZAtJCFpu8Ir1m4kpw")
GGA_fig = px.scatter_mapbox(GGA_df, lat="Val_2", lon="Val_3", color="Time", size=[x+1 for x in range(len(GGA_df))],
                            color_continuous_scale=px.colors.sequential.OrRd, zoom=17, mapbox_style="satellite", title="GGA Coordinates Plot")
GGA_fig.show()


####################
#     RMC Plot
####################
px.set_mapbox_access_token(
    "pk.eyJ1IjoiZm9sa2lzaGFjb3JuIiwiYSI6ImNsM2xxNWY3dTAzdDkzY284Z2dnbWZlYm0ifQ.gtXNkZAtJCFpu8Ir1m4kpw")
RMC_fig = px.scatter_mapbox(RMC_df, lat="Val_2", lon="Val_3", color="Time", size=[x+1 for x in range(len(GGA_df))],
                            color_continuous_scale=px.colors.sequential.OrRd, zoom=17, mapbox_style="satellite", title="RMC Coordinates Plot")
RMC_fig.show()


####################
#     ACC Plot
####################
ACC_x = ACC_df["Val_1"]
ACC_y = ACC_df["Val_2"]
ACC_z = ACC_df["Val_3"]
ACC_time = ACC_df["Time"]

ACC_axs = []
ACC_fig, ACC_axs = plt.subplots(3, sharex=True, sharey=True)

ACC_axs[0].plot(ACC_time, ACC_x)
ACC_axs[0].set_ylabel("Acc x")

ACC_axs[1].plot(ACC_time, ACC_y, color="blue")
ACC_axs[1].set_ylabel("Acc y")

ACC_axs[2].plot(ACC_time, ACC_z, color="green")
ACC_axs[2].set_ylabel("Acc z")
ACC_axs[2].set_xlabel("Time (s)")

ACC_fig.suptitle("High G Accelerometer Plots")

####################
#    SPEC Plot
####################
SPEC_1 = SPEC_df["Val_1"]
SPEC_2 = SPEC_df["Val_2"]
SPEC_time = SPEC_df["Time"]

SPEC_fig, SPEC_axs = plt.subplots()
SPEC_axs.set_ylim([0, 1])
SPEC_axs.scatter(SPEC_time, SPEC_1, label="Spectrometer 1")
SPEC_axs.scatter(SPEC_time, SPEC_2, label="Spectrometer 2")
SPEC_axs.legend()

SPEC_axs.set_xlabel("Time (s)")

SPEC_fig.suptitle("Spectrometer Ratio Plots")

####################
#    Misc Plot
####################
hdop = GGA_df_1["Val_4"]
alt = GGA_df_1["Val_5"]
GGA_time = GGA_df_1["Time"]
speed = RMC_df_1["Val_4"]
course = RMC_df_1["Val_5"]
RMC_time = RMC_df_1["Time"]

misc_axs = []
misc_fig, misc_axs = plt.subplots(2, 2, sharex=True)
misc_axs[0, 0].scatter(GGA_time, hdop)
misc_axs[0, 0].set_ylabel("Hdop")
misc_axs[1, 0].plot(GGA_time, alt, color="blue")
misc_axs[1, 0].set_ylabel("Altitude")
misc_axs[0, 1].plot(RMC_time, speed, color="green")
misc_axs[0, 1].set_ylabel("Speed")
misc_axs[1, 1].scatter(RMC_time, course, color="#8B8000")
misc_axs[1, 1].set_ylabel("Course")

misc_axs[0, 0].set_title("GGA")
misc_axs[0, 1].set_title("RMC")
misc_axs[1, 0].set_xlabel("Time (s)")
misc_axs[1, 1].set_xlabel("Time (s)")


plt.show()
