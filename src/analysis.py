import numpy as np
class PCA:
    """
    Principal Component Analysis (PCA) class for dimensionality reduction.
    """
    
    def __init__(self, n_components):
        """
        Constructor method that initializes the PCA object with the number of components to retain.
        
        Args:
        - n_components (int): Number of principal components to retain.
        """
        self.n_components = n_components
        
    def fit(self, X):
        """
        Fits the PCA model to the input data and computes the principal components.
        
        Args:
        - X (numpy.ndarray): Input data matrix with shape (n_samples, n_features).
        """
        # Compute the mean of the input data along each feature dimension.
        mean = np.mean(X, axis=0)
        
        # Subtract the mean from the input data to center it around zero.
        X = X - mean
        
        # Compute the covariance matrix of the centered input data.
        cov = np.cov(X.T)
        
        # Compute the eigenvectors and eigenvalues of the covariance matrix.
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # Reverse the order of the eigenvalues and eigenvectors.
        eigenvalues = eigenvalues[::-1]
        eigenvectors = eigenvectors[:,::-1]
        
        # Keep only the first n_components eigenvectors as the principal components.
        self.components = eigenvectors[:,:self.n_components]
        
        # Compute the explained variance ratio for each principal component.
        # Compute the total variance of the input data
        total_variance = np.sum(np.var(X, axis=0))

        # Compute the variance explained by each principal component
        self.explained_variances = eigenvalues[:self.n_components]

        # Compute the explained variance ratio for each principal component
        self.explained_variance_ratio_ = self.explained_variances / total_variance
        
    def transform(self, X):
        """
        Transforms the input data by projecting it onto the principal components.
        
        Args:
        - X (numpy.ndarray): Input data matrix with shape (n_samples, n_features).
        
        Returns:
        - transformed_data (numpy.ndarray): Transformed data matrix with shape (n_samples, n_components).
        """
        # Center the input data around zero using the mean computed during the fit step.
        X = X - np.mean(X, axis=0)
        
        # Project the centered input data onto the principal components.
        transformed_data = np.dot(X, self.components)
        
        return transformed_data
    
    def fit_transform(self, X):
        """
        Fits the PCA model to the input data and computes the principal components then
        transforms the input data by projecting it onto the principal components.
        
        Args:
        - X (numpy.ndarray): Input data matrix with shape (n_samples, n_features).
        """
        self.fit(X)
        transformed_data = self.transform(X)
        return transformed_data