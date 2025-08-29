import numpy as np
from sklearn.ensemble import RandomForestRegressor
import pickle
from sklearn.model_selection import train_test_split

class GridPatternPredictor:
    """
    Production-ready Random Forest predictor for finding (x, y) coordinates
    in 2D grid patterns.
    """
    def __init__(self, n_estimators=50, random_state=42):
        self.model = RandomForestRegressor(
            n_estimators=n_estimators,
            random_state=random_state,
            max_depth=8,
            min_samples_split=3,
            min_samples_leaf=1,
            n_jobs=-1,
            bootstrap=True,
            oob_score=True
        )
        self.grid_shape = None
        self.is_trained = False

    def train(self, grids, coords):
        """
        Train the Random Forest on your augmented data.

        Parameters:
        -----------
        grids : list of numpy arrays
            Each array is a 2D grid input.
        coords : list of [x, y]
            Corresponding target coordinates for each grid.
        """
        # Flatten grids into feature vectors
        X = np.array([grid.flatten() for grid in grids])
        y = np.array(coords)

        # Store grid shape
        self.grid_shape = grids[0].shape

        # Fit the model
        self.model.fit(X, y)
        self.is_trained = True

        print(f"Training complete. OOB score: {self.model.oob_score_:.4f}")

    def predict(self, grid):
        """
        Predict the (x, y) coordinate for a single grid.

        Parameters:
        -----------
        grid : numpy array
            A single 2D grid.

        Returns:
        --------
        (x, y) tuple of ints
        """
        if not self.is_trained:
            raise RuntimeError("Model must be trained before prediction.")

        features = grid.flatten().reshape(1, -1)
        pred = self.model.predict(features)[0]

        # Round and clamp to valid indices
        x = int(np.clip(round(pred[0]), None, self.grid_shape[0] - 1))
        y = int(np.clip(round(pred[1]), None, self.grid_shape[1] - 1))
        return x, y

    def batch_predict(self, grids):
        """
        Predict coordinates for a list of grids.

        Parameters:
        -----------
        grids : list of numpy arrays

        Returns:
        --------
        List of (x, y) tuples
        """
        if not self.is_trained:
            raise RuntimeError("Model must be trained before prediction.")

        X = np.array([g.flatten() for g in grids])
        preds = self.model.predict(X)
        results = []
        for pred in preds:
            x = int(np.clip(round(pred[0]), None, self.grid_shape[0] - 1))
            y = int(np.clip(round(pred[1]), None, self.grid_shape[1] - 1))
            results.append((x, y))
        return results

    def save(self, filepath):
        """
        Save the trained model to disk.
        """
        if not self.is_trained:
            raise RuntimeError("No trained model to save.")
        with open(filepath, 'wb') as f:
            pickle.dump(self, f)
        print(f"Model saved to {filepath}")

    @staticmethod
    def load(filepath):
        """
        Load a saved model from disk.
        """
        with open(filepath, 'rb') as f:
            model = pickle.load(f)
        print(f"Model loaded from {filepath}")
        return model

# # ------------------------
# # Example Usage:

# # 1. Prepare your data
# # Load augmented dataset
# with open('training_data/input/augmented_grids2.pkl', 'rb') as f:
#     augmented_dataset = pickle.load(f)

# # # Find largest grid dimensions
# # max_rows = max(grid.shape[0] for grid in augmented_dataset)
# # max_cols = max(grid.shape[1] for grid in augmented_dataset)

# # # Pad all grids to largest dimension with value 1
# # padded_dataset = []
# # for grid in augmented_dataset:
# #     pad_rows = max_rows - grid.shape[0]
# #     pad_cols = max_cols - grid.shape[1]
# #     padded_grid = np.pad(grid, ((0, pad_rows), (0, pad_cols)), mode='constant', constant_values=1)
# #     padded_dataset.append(padded_grid)

# # augmented_dataset = padded_dataset

# # Load corresponding coordinates
# with open('training_data/input/augmented_coords2.pkl', 'rb') as f:
#     augmented_coords = pickle.load(f)

# # Split data into training and test sets (e.g., 80% train, 20% test)
# X_train, X_test, y_train, y_test = train_test_split(
#     augmented_dataset, augmented_coords, test_size=0.2, random_state=42
# )

# # 2. Train the model
# predictor = GridPatternPredictor()
# predictor.train(X_train, y_train)

# #3. Predict on new grid
# # # Predict on test dataset and show first 5 predicted and real values
# # test_preds = predictor.batch_predict(X_test)
# # for i in range(10):
# #     print(f"Predicted: {test_preds[i]}, Actual: {y_test[i]}")

# # Load original grids and coordinates
# with open('training_data/input/original_grids.pkl', 'rb') as f:
#     original_grids = pickle.load(f)
# with open('training_data/input/original_coords.pkl', 'rb') as f:
#     original_coords = pickle.load(f)

# # # Pad original grids to match training grid shape
# # padded_original_grids = []
# # for grid in original_grids:
# #     pad_rows = predictor.grid_shape[0] - grid.shape[0]
# #     pad_cols = predictor.grid_shape[1] - grid.shape[1]
# #     padded_grid = np.pad(grid, ((0, pad_rows), (0, pad_cols)), mode='constant', constant_values=1)
# #     padded_original_grids.append(padded_grid)

# # Predict on padded original grids
# original_preds = predictor.batch_predict(original_grids)

# # Display predicted and real coordinates
# for i in range(len(original_preds)):
#     print(f"Predicted: {original_preds[i]}, Actual: {original_coords[i]}")



# # # 4. Save model
# predictor.save('grid_predictor_working_augmented.pkl')

# # 5. Load model later
# # predictor = GridPatternPredictor.load('grid_predictor.pkl')
