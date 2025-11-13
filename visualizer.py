"""
Visualizer Module
Generates and displays plots/heatmaps for data insights
"""

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


class Visualizer:
    """Generates visualizations for analysis results"""
    
    def __init__(self):
        pass
        
    def plot_trajectory(self, trajectory, obstacles=None, figure=None):
        """
        Plot vehicle trajectory with obstacles
        
        Args:
            trajectory (list): List of (x, y, theta) tuples
            obstacles (list): Optional list of obstacle positions
            figure (Figure): Matplotlib figure to plot on
        """
        if figure is None:
            fig, ax = plt.subplots(figsize=(10, 8))
        else:
            fig = figure
            ax = self._setup_axes(figure)
            
        # Extract x, y coordinates
        if trajectory:
            x_coords = [pose[0] for pose in trajectory]
            y_coords = [pose[1] for pose in trajectory]
            
            # Plot trajectory
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, label='Trajectory')
            ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')
            ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')
            
            # Plot obstacles if provided
            if obstacles:
                obs_x = [obs[0] for obs in obstacles]
                obs_y = [obs[1] for obs in obstacles]
                ax.scatter(obs_x, obs_y, c='red', s=100, marker='x', label='Obstacles')
                
            ax.set_xlabel('X Position (m)')
            ax.set_ylabel('Y Position (m)')
            ax.set_title('Vehicle Trajectory')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            
        return fig
        
    def plot_heatmap(self, corr, figure=None):
        """
        Render correlation matrix heatmap
        
        Args:
            corr (pd.DataFrame): Correlation matrix
            figure (Figure): Matplotlib figure to plot on
        """
        if figure is None:
            fig, ax = plt.subplots(figsize=(10, 8))
        else:
            fig = figure
            ax = self._setup_axes(figure)
            
        sns.heatmap(corr, annot=True, fmt='.2f', cmap='RdYlGn', 
                   center=0, ax=ax, cbar_kws={'label': 'Correlation'},
                   vmin=-1, vmax=1)
        ax.set_title('Correlation Heatmap')
        
        return fig
        
    def plot_time_series(self, df, columns, figure=None):
        """
        Plot time series data
        
        Args:
            df (pd.DataFrame): Data with time series
            columns (list): Columns to plot
            figure (Figure): Matplotlib figure to plot on
        """
        if figure is None:
            fig, ax = plt.subplots(figsize=(12, 6))
        else:
            fig = figure
            ax = self._setup_axes(figure)
            
        for col in columns:
            if col in df.columns:
                ax.plot(df.index, df[col], label=col)
                
        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.set_title('Time Series Plot')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        return fig
        
    def plot_anomalies(self, df, anomaly_indices, column, figure=None):
        """
        Plot data with anomalies highlighted
        
        Args:
            df (pd.DataFrame): Original data
            anomaly_indices (list): Indices of anomalies
            column (str): Column to plot
            figure (Figure): Matplotlib figure to plot on
        """
        if figure is None:
            fig, ax = plt.subplots(figsize=(12, 6))
        else:
            fig = figure
            ax = self._setup_axes(figure)
            
        # Plot normal data
        ax.plot(df.index, df[column], 'b-', alpha=0.6, label='Normal')
        
        # Highlight anomalies
        if anomaly_indices:
            ax.scatter(anomaly_indices, df.loc[anomaly_indices, column],
                      c='red', s=100, marker='o', label='Anomalies')
                      
        ax.set_xlabel('Index')
        ax.set_ylabel(column)
        ax.set_title(f'Anomaly Detection: {column}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        return fig
        
    def _setup_axes(self, figure):
        """
        Configure plot axes with labels
        
        Args:
            figure (Figure): Matplotlib figure
            
        Returns:
            Axes: Configured axes
        """
        figure.clear()
        ax = figure.add_subplot(111)
        return ax
        
    def create_comparison_plot(self, before_df, after_df, column, figure=None):
        """
        Create before/after comparison plot
        
        Args:
            before_df (pd.DataFrame): Data before injection
            after_df (pd.DataFrame): Data after injection
            column (str): Column to compare
            figure (Figure): Matplotlib figure to plot on
        """
        if figure is None:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        else:
            fig = figure
            fig.clear()
            ax1 = fig.add_subplot(121)
            ax2 = fig.add_subplot(122)
            
        # Before injection
        ax1.plot(before_df.index, before_df[column], 'b-')
        ax1.set_title('Before Injection')
        ax1.set_xlabel('Index')
        ax1.set_ylabel(column)
        ax1.grid(True, alpha=0.3)
        
        # After injection
        ax2.plot(after_df.index, after_df[column], 'r-')
        ax2.set_title('After Injection')
        ax2.set_xlabel('Index')
        ax2.set_ylabel(column)
        ax2.grid(True, alpha=0.3)
        
        fig.suptitle(f'Anomaly Injection Comparison: {column}')
        fig.tight_layout()
        
        return fig
