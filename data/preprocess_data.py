import pandas as pd
from sklearn.model_selection import train_test_split

def preprocess_data(input_file, output_dir):
    # Load dataset
    data = pd.read_csv(input_file)
    # Clean and preprocess
    data = data.dropna().reset_index(drop=True)
    # Split into train and test sets
    train, test = train_test_split(data, test_size=0.2, random_state=42)
    # Save processed data
    train.to_csv(f"{output_dir}/train.csv", index=False)
    test.to_csv(f"{output_dir}/test.csv", index=False)

# Example usage
if __name__ == "__main__":
    preprocess_data("raw_data.csv", "processed_data")
