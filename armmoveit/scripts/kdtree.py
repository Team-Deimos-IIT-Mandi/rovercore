import joblib
import numpy as np
import os

def debug_tree_files(tree_path, npz_path):
    print("--- Starting KDTree/NPZ Debug ---")
    
    # 1. Check if files exist
    if not os.path.exists(tree_path):
        print(f"Error: Tree file not found at {tree_path}")
        return
    if not os.path.exists(npz_path):
        print(f"Error: NPZ file not found at {npz_path}")
        return

    # 2. Inspect the KDTree
    print(f"Loading tree from {tree_path}...")
    tree = joblib.load(tree_path)
    print(f"Tree Type: {type(tree)}")
    # KDTree objects don't always have a __len__, but they have n and m
    print(f"Number of points in tree (n): {tree.n}")
    print(f"Dimensionality of points (m): {tree.m}")

    # 3. Inspect the NPZ Angles Array
    print(f"\nLoading angles from {npz_path}...")
    data = np.load(npz_path)
    # List files in the NPZ to ensure 'angles' key exists
    print(f"Available keys in NPZ: {list(data.keys())}")
    
    if 'angles' in data:
        angles = data['angles']
        print(f"Angles Array Shape: {angles.shape}")
        print(f"Angles Array Size: {angles.size}")
        print(f"First element: {angles[0]}")
        
        # 4. Verify cross-compatibility
        if tree.n == angles.shape[0]:
            print("\nSUCCESS: Tree points and Angles array length match.")
        else:
            print(f"\nCRITICAL ERROR: Tree points ({tree.n}) != Angles rows ({angles.shape[0]})")
    else:
        print("Error: Key 'angles' not found in NPZ file.")

if __name__ == "__main__":
    # Change this line in your debug script:
    debug_tree_files('/workspace/workspace_tree.joblib', '/workspace/workspace_lookup_sampled.npz')