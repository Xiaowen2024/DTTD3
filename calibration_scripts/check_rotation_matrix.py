import numpy as np

def is_valid_rotation_matrix(R, tol=1e-6):
    # Check if R * R^T is close to the identity matrix
    orthogonality_check = np.allclose(np.dot(R, R.T), np.identity(3), atol=tol)
    
    # Check if the determinant is close to +1
    determinant_check = np.isclose(np.linalg.det(R), 1.0, atol=tol)
    
    # Check if each row and each column has a unit norm
    row_norms = np.linalg.norm(R, axis=1)
    column_norms = np.linalg.norm(R, axis=0)
    unit_norms_check = np.allclose(row_norms, 1, atol=tol) and np.allclose(column_norms, 1, atol=tol)
    
    # Return overall validity and individual checks
    is_valid = orthogonality_check and determinant_check and unit_norms_check
    return {
        "is_valid": is_valid,
        "orthogonality_check": orthogonality_check,
        "determinant_check": determinant_check,
        "unit_norms_check": unit_norms_check
    }

# Example usage
R = np.array([
    [0.09079261, -0.82169971,  0.56264224],
    [0.72849922,  0.44001228,  0.52505055],
    [-0.67900337, 0.36221372,  0.63855747]
])

result = is_valid_rotation_matrix(R)
print(result)
