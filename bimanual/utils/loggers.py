import os
import pandas as pd


def write_to_csv(csv_file: str, df: pd.DataFrame):

    # if csv_file does not exist, create it
    if not os.path.exists(csv_file):
        with open(csv_file, "x") as f:
            df.to_csv(f, index=False, header=True)
        return

    else:
        # if csv_file exists, check if columns match
        with open(csv_file, "r") as f:
            header = f.readline()
            # Gets list of column names in csv. Removes newline and splits by comma.
            header_columns = header.strip('\n').split(",")
            if header_columns != df.columns.to_list():
                raise ValueError(
                    f"Columns do not match: {header} != {df.columns}")

        # if columns match, append to csv
        with open(csv_file, "a") as f:
            df.to_csv(f, index=False, header=False)
