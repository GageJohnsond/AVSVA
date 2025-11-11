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
        Compute correlation matrix from DataFrames by merging on timestamp

        Args:
            dfs (dict): Dictionary of DataFrames (from different topics)

        Returns:
            pd.DataFrame: Correlation matrix
        """
        if not dfs:
            return pd.DataFrame()

        # Merge all DataFrames on timestamp
        merged_df = self._merge_dataframes(dfs)

        if merged_df.empty:
            return pd.DataFrame()

        # Select only numeric columns for correlation
        numeric_cols = merged_df.select_dtypes(include=[np.number]).columns
        numeric_cols = [col for col in numeric_cols if col != 'timestamp']

        if len(numeric_cols) < 2:
            return pd.DataFrame()

        # Compute correlation matrix
        corr_matrix = merged_df[numeric_cols].corr()

        return corr_matrix

    def _merge_dataframes(self, dfs):
        """
        Merge multiple DataFrames on timestamp with tolerance

        Args:
            dfs (dict): Dictionary of DataFrames

        Returns:
            pd.DataFrame: Merged DataFrame
        """
        if not dfs:
            return pd.DataFrame()

        # Convert dict values to list
        df_list = list(dfs.values())

        if len(df_list) == 0:
            return pd.DataFrame()

        if len(df_list) == 1:
            return df_list[0]

        # Start with first DataFrame
        merged = df_list[0].copy()

        # Ensure timestamp column exists
        if 'timestamp' not in merged.columns:
            return merged

        # Merge with remaining DataFrames
        for i, df in enumerate(df_list[1:], 1):
            if 'timestamp' in df.columns:
                # Merge on timestamp with tolerance using merge_asof
                merged = pd.merge_asof(
                    merged.sort_values('timestamp'),
                    df.sort_values('timestamp'),
                    on='timestamp',
                    direction='nearest',
                    tolerance=0.1,  # 100ms tolerance
                    suffixes=('', f'_{i}')
                )

        return merged
        
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
            'count': len(df[column]),
            'q25': df[column].quantile(0.25),
            'q75': df[column].quantile(0.75)
        }

    def detect_multi_column_anomalies(self, dfs, columns=None, threshold=3.0):
        """
        Detect anomalies across multiple columns/topics

        Args:
            dfs (dict): Dictionary of DataFrames
            columns (list): Specific columns to check (None = auto-detect numeric)
            threshold (float): Z-score threshold

        Returns:
            pd.DataFrame: Anomalies with timestamps and affected columns
        """
        merged_df = self._merge_dataframes(dfs)

        if merged_df.empty:
            return pd.DataFrame()

        # Select columns to analyze
        if columns is None:
            columns = merged_df.select_dtypes(include=[np.number]).columns
            columns = [col for col in columns if col != 'timestamp']

        anomalies_list = []

        for col in columns:
            if col in merged_df.columns:
                z_scores = self._compute_zscore(merged_df[col])
                anomaly_mask = np.abs(z_scores) > threshold

                if anomaly_mask.any():
                    anomaly_df = merged_df[anomaly_mask].copy()
                    anomaly_df['anomaly_column'] = col
                    anomaly_df['z_score'] = z_scores[anomaly_mask]
                    anomalies_list.append(anomaly_df)

        if anomalies_list:
            return pd.concat(anomalies_list, ignore_index=True)

        return pd.DataFrame()

    def compute_signal_quality(self, df, column):
        """
        Compute signal quality metrics for a data column

        Args:
            df (pd.DataFrame): Input DataFrame
            column (str): Column to analyze

        Returns:
            dict: Signal quality metrics
        """
        if df is None or df.empty or column not in df.columns:
            return {}

        data = df[column].dropna()

        if len(data) == 0:
            return {'error': 'No valid data'}

        # Compute various quality metrics
        quality = {
            'completeness': len(data) / len(df[column]) * 100,  # % non-null
            'variance': data.var(),
            'range': data.max() - data.min(),
            'snr_estimate': data.mean() / data.std() if data.std() > 0 else float('inf')
        }

        # Detect gaps in timestamp if available
        if 'timestamp' in df.columns:
            timestamps = df['timestamp'].dropna().sort_values()
            if len(timestamps) > 1:
                time_diffs = timestamps.diff().dropna()
                quality['avg_sample_rate'] = 1.0 / time_diffs.mean() if time_diffs.mean() > 0 else 0
                quality['max_gap'] = time_diffs.max()
                quality['timing_jitter'] = time_diffs.std()

        return quality

    def detect_sensor_failures(self, dfs):
        """
        Detect potential sensor failures across topics

        Args:
            dfs (dict): Dictionary of DataFrames

        Returns:
            dict: Detected failures by topic/column
        """
        failures = {}

        for topic_name, df in dfs.items():
            if df.empty:
                failures[topic_name] = {'error': 'Empty dataset'}
                continue

            topic_failures = []

            # Check numeric columns for issues
            numeric_cols = df.select_dtypes(include=[np.number]).columns

            for col in numeric_cols:
                if col == 'timestamp':
                    continue

                data = df[col].dropna()

                if len(data) == 0:
                    topic_failures.append({
                        'column': col,
                        'failure_type': 'no_data',
                        'description': 'All values are null/NaN'
                    })
                    continue

                # Detect stuck values (all same)
                if data.nunique() == 1:
                    topic_failures.append({
                        'column': col,
                        'failure_type': 'stuck_value',
                        'description': f'All values equal to {data.iloc[0]}'
                    })

                # Detect excessive nulls
                null_pct = (df[col].isna().sum() / len(df)) * 100
                if null_pct > 50:
                    topic_failures.append({
                        'column': col,
                        'failure_type': 'excessive_nulls',
                        'description': f'{null_pct:.1f}% null values'
                    })

                # Detect extreme variance changes
                if len(data) > 10:
                    half = len(data) // 2
                    var1 = data.iloc[:half].var()
                    var2 = data.iloc[half:].var()
                    if var1 > 0 and var2 > 0:
                        var_ratio = max(var1, var2) / min(var1, var2)
                        if var_ratio > 100:
                            topic_failures.append({
                                'column': col,
                                'failure_type': 'variance_shift',
                                'description': f'Variance changed by {var_ratio:.1f}x'
                            })

            if topic_failures:
                failures[topic_name] = topic_failures

        return failures
