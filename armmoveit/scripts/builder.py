import numpy as np
from scipy.spatial import KDTree
import joblib

# 1. Load your raw lookup data
data = np.load('/workspace/workspace_lookup.npz')
pts = data['points']
angs = data['angles']

# 2. STRATEGIC DOWNSAMPLING
# Take every 30th point. This keeps the shape of the shell 
# but reduces the tree size to 1 million points.
SAMPLE_RATE = 30 
pts_sampled = pts[::SAMPLE_RATE]
angs_sampled = angs[::SAMPLE_RATE]

print(f"Building tree with {len(pts_sampled)} points instead of {len(pts)}...")

# 3. Build the tree
tree = KDTree(pts_sampled)

# 4. Save the tree AND the sampled angles
joblib.dump(tree, '/workspace/workspace_tree.joblib', compress=3)
np.savez_compressed('/workspace/workspace_lookup_sampled.npz', angles=angs_sampled)

print("Optimized sampled structure saved.")