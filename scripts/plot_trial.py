# import seaborn as sns
# import pandas as pd
# import matplotlib.pyplot as plt

# # Load the 'tips' dataset
# data = sns.load_dataset('tips')

# # Calculate min, max, and average for each day
# data_summary = data.groupby('day').agg({'total_bill': ['min', 'max', 'mean']})
# data_summary.columns = data_summary.columns.droplevel(0)

# # Reset the index
# data_summary.reset_index(inplace=True)

# # Melt the DataFrame to create a 'tidy' format for plotting
# data_melted = data_summary.melt(id_vars='day', var_name='stat', value_name='total_bill')

# # Create the line plot
# plot = sns.lineplot(data=data_melted, x='day', y='total_bill', hue='stat', marker='o', linewidth=2)

# # Customize the plot
# plt.title("Min, Max, and Average Total Bill per Day")
# plt.xlabel("Day of the Week")
# plt.ylabel("Total Bill")

# # Add shaded areas between min and max
# days = data_summary['day'].unique()
# for day in days:
#     day_data = data_summary[data_summary['day'] == day]
#     plt.fill_between([day], day_data['min'], day_data['max'], color='gray', alpha=0.3)


import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


# import seaborn as sns

# https://seaborn.pydata.org/tutorial/color_palettes.html#other-diverging-palettes


# # Generate a color palette with 10 colors
# palette = sns.color_palette("Spectral", n_colors=10)

# # Print the color palette
# print("Color palette:")
# for i, color in enumerate(palette):
#     print(f"Color {i}: {color}")

# # Pick specific colors from the palette
# color_3 = palette[2]
# color_6 = palette[5]

# print("\nSelected colors:")
# print(f"Color 3: {color_3}")
# print(f"Color 6: {color_6}")


# Create a sample dataset with date, min, max, and average values
data = pd.DataFrame({
    'date': pd.date_range(start='2023-01-01', periods=10, freq='D'),
    'min': [4, 3, 5, 6, 4, 3, 5, 6, 7, 8],
    'max': [10, 9, 11, 12, 10, 9, 11, 12, 13, 14],
    'mean': [7, 6, 8, 9, 7, 6, 8, 9, 10, 11]
})

palette = sns.color_palette("Spectral", n_colors=10)
# Print the color palette
print("Color palette:")
for i, color in enumerate(palette):
    print(f"Color {i}: {color}")


sns.color_palette("Spectral", as_cmap=True)

# Set the figure size
plt.figure(figsize=(10, 6))

# Plot the average values using Seaborn lineplot
sns.lineplot(data=data, x='date', y='mean', label='Average', color=palette[2], linewidth=2)

# Plot the shaded area using Matplotlib fill_between
plt.fill_between(data['date'], data['min'], data['max'], color=palette[2], alpha=0.1)

# Customize the plot
plt.title("Time Series Plot with Min, Max, and Average Values")
plt.xlabel("Date")
plt.ylabel("Value")
plt.legend()

# Show the plot
plt.savefig('try.png')