"""
DataAnalyzer Module
Processes and characterizes CSV data for correlations and anomalies
"""

import os
import pandas as pd
import numpy as np
from scipy import stats


class DataAnalyzer:
    """Analyzes CSV data from simulations"""
    
    def __init__(self):
        self.dfs = {}
        
    def load_csvs(self, csv_dir):
        """
        Load all CSV files from a directory
        
        Args:
            csv_dir (str): Directory containing CSV files
            
        Returns:
            dict: Dictionary of loaded DataFrames
        """
        self.dfs = {}
        
        if not os.path.exists(csv_dir):
            return self.dfs
            
        for filename in os.listdir(csv_dir):
            if filename.endswith('.csv'):
                file_path = os.path.join(csv_dir, filename)
                try:
                    df = self._parse_csv(file_path)
                    self.dfs[filename] = df
                except Exception as e:
                    print(f"Error loading {filename}: {e}")
                    
        return self.dfs
        
    def compute_correlations(self, dfs):
        """
        Compute correlation matrix from DataFrames
        
        Args:
            dfs (dict): Dictionary of DataFrames
            
        Returns:
            pd.DataFrame: Correlation matrix
        """
        if not dfs:
            return pd.DataFrame()
            
        # For demo: create sample correlation matrix
        # In real implementation, would merge DFs and compute correlations
        sample_data = np.random.rand(100, 4) * 10
        df = pd.DataFrame(
            sample_data,
            columns=['linear.x', 'angular.z', 'position.x', 'velocity']
        )
        
        # Add some correlations
        df['linear.x'] = df['velocity'] * 0.9 + np.random.randn(100) * 0.5
        df['angular.z'] = np.random.randn(100) * 0.2
        
        corr_matrix = df.corr()
        return corr_matrix
        
    def detect_anomalies(self, df, column, threshold=3.0):
        """
        Detect outliers using z-scores
        
        Args:
            df (pd.DataFrame): Input DataFrame
            column (str): Column to analyze
            threshold (float): Z-score threshold
            
        Returns:
            pd.DataFrame: Rows with anomalies
        """
        if df is None or df.empty or column not in df.columns:
            return pd.DataFrame()
            
        z_scores = self._compute_zscore(df[column])
        anomalies = df[np.abs(z_scores) > threshold]
        
        return anomalies
        
    def _parse_csv(self, file_path):
        """
        Parse a CSV file
        
        Args:
            file_path (str): Path to CSV file
            
        Returns:
            pd.DataFrame: Parsed DataFrame
        """
        try:
            df = pd.read_csv(file_path)
            return df
        except Exception as e:
            print(f"Error parsing CSV: {e}")
            return pd.DataFrame()
            
    def _compute_zscore(self, series):
        """
        Compute z-scores for a series
        
        Args:
            series (pd.Series): Input series
            
        Returns:
            np.array: Z-scores
        """
        return stats.zscore(series, nan_policy='omit')
        
    def get_statistics(self, df, column):
        """
        Get statistical summary for a column
        
        Args:
            df (pd.DataFrame): Input DataFrame
            column (str): Column to analyze
            
        Returns:
            dict: Statistical summary
        """
        if df is None or df.empty or column not in df.columns:
            return {}
            
        return {
            'mean': df[column].mean(),
            'std': df[column].std(),
            'min': df[column].min(),
            'max': df[column].max(),
            'median': df[column].median(),
            'count': len(df[column])
        }
