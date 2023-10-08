from sklearn import tree

all_colors = [
    # "BLACK",
    "BLUE",
    "BROWN",
    "GREEN",
    "RED",
    # "WHITE",
    # "YELLOW",
]

s1_files = [
    "calibration_sensor_Port.S1_Color.BLACK_2020_04_11_02_39_39_117926.csv",
    "calibration_sensor_Port.S1_Color.BLUE_2020_04_11_02_38_00_002935.csv",
    "calibration_sensor_Port.S1_Color.BROWN_2020_04_11_02_39_22_229294.csv",
    "calibration_sensor_Port.S1_Color.GREEN_2020_04_11_02_39_05_486703.csv",
    "calibration_sensor_Port.S1_Color.RED_2020_04_11_02_38_48_567920.csv",
    "calibration_sensor_Port.S1_Color.WHITE_2020_04_11_02_40_17_819093.csv",
    "calibration_sensor_Port.S1_Color.YELLOW_2020_04_11_02_39_57_169476.csv",
]

s2_files = [
    "calibration_sensor_Port.S2_Color.BLACK_2020_04_11_02_41_53_048756.csv",
    "calibration_sensor_Port.S2_Color.BLUE_2020_04_11_02_40_34_159722.csv",
    "calibration_sensor_Port.S2_Color.BROWN_2020_04_11_02_41_35_709681.csv",
    "calibration_sensor_Port.S2_Color.GREEN_2020_04_11_02_41_16_269288.csv",
    "calibration_sensor_Port.S2_Color.RED_2020_04_11_02_40_57_419683.csv",
    "calibration_sensor_Port.S2_Color.WHITE_2020_04_11_02_42_31_467700.csv",
    "calibration_sensor_Port.S2_Color.YELLOW_2020_04_11_02_42_12_839364.csv",
]

s3_files = [
    "calibration_sensor_Port.S3_Color.BLACK_2020_04_11_02_44_48_430834.csv",
    "calibration_sensor_Port.S3_Color.BLUE_2020_04_11_02_42_47_171605.csv",
    "calibration_sensor_Port.S3_Color.BROWN_2020_04_11_02_44_29_901280.csv",
    "calibration_sensor_Port.S3_Color.GREEN_2020_04_11_02_44_03_621428.csv",
    "calibration_sensor_Port.S3_Color.RED_2020_04_11_02_43_43_482714.csv",
    "calibration_sensor_Port.S3_Color.WHITE_2020_04_11_02_45_26_831859.csv",
    "calibration_sensor_Port.S3_Color.YELLOW_2020_04_11_02_45_09_562001.csv",
]

s4_files = [
    "calibration_sensor_Port.S4_Color.BLACK_2020_04_11_02_46_55_993712.csv",
    "calibration_sensor_Port.S4_Color.BLUE_2020_04_11_02_45_43_567235.csv",
    "calibration_sensor_Port.S4_Color.BROWN_2020_04_11_02_46_38_110439.csv",
    "calibration_sensor_Port.S4_Color.GREEN_2020_04_11_02_46_20_202005.csv",
    "calibration_sensor_Port.S4_Color.RED_2020_04_11_02_46_02_280880.csv",
    "calibration_sensor_Port.S4_Color.WHITE_2020_04_11_02_47_52_781659.csv",
    "calibration_sensor_Port.S4_Color.YELLOW_2020_04_11_02_47_12_911875.csv",
]


momo_s3_files = [
    "momo_calibration_sensor_Port.S3_Color.BLUE_2020_04_18_11_32_37_127387.csv",
    "momo_calibration_sensor_Port.S3_Color.BROWN_2020_04_18_09_02_59_636320.csv",
    "momo_calibration_sensor_Port.S3_Color.GREEN_2020_04_18_11_34_11_386542.csv",
    "momo_calibration_sensor_Port.S3_Color.RED_2020_04_18_11_33_25_374582.csv",
]

color_array = []
for filename in momo_s3_files:
    with open("./logs/" + filename) as f:
        for line in f:
            color_array.append(
                tuple(
                    [int(x.replace("(", "").replace(")", "")) for x in line.split(",")]
                )
            )


print(color_array)
print(len(color_array))

decision_tree = tree.DecisionTreeClassifier()
decision_tree = decision_tree.fit(
    color_array, [w for sublist in [[e] * 100 for e in all_colors] for w in sublist]
)


print(tree.export_text(decision_tree, feature_names=["R", "G", "B"]))
