import pandas as pd

def write_to_csv(csv_file: str, df: pd.DataFrame):
    try:
        with open(csv_file, "x") as f:
            df.to_csv(f, index=False, header=True)
    except FileExistsError:
        # TODO: Check if the columns match
        pass

    with open(csv_file, "a") as f:
        df.to_csv(f, index=False, header=False)
