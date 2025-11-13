"""
ReportGenerator Module
Compiles summaries, tables, and findings into reports
"""

from datetime import datetime
import pandas as pd


class ReportGenerator:
    """Generates analysis and vulnerability reports"""
    
    def __init__(self):
        pass
        
    def generate_report(self, correlations=None, anomalies=None, failures=None):
        """
        Generate comprehensive analysis report
        
        Args:
            correlations (pd.DataFrame): Correlation matrix
            anomalies (pd.DataFrame): Detected anomalies
            failures (str): Failure state descriptions
            
        Returns:
            str: Formatted report text
        """
        report = []
        report.append("=" * 80)
        report.append("AUTONOMOUS VEHICLE SIMULATION AND VULNERABILITY ANALYSIS REPORT")
        report.append("=" * 80)
        report.append(f"\nGenerated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("\n")
        
        # Executive Summary
        report.append("EXECUTIVE SUMMARY")
        report.append("-" * 80)
        report.append("This report summarizes the analysis of autonomous vehicle simulation data,")
        report.append("including data characterization, anomaly detection, and vulnerability testing.")
        report.append("\n")
        
        # Correlation Analysis
        if correlations is not None and not correlations.empty:
            report.append("CORRELATION ANALYSIS")
            report.append("-" * 80)
            report.append("Correlation matrix computed from sensor data:")
            report.append("\n")
            report.append(self._format_table(correlations))
            report.append("\n")
            report.append("Key Findings:")
            
            # Find strongest correlations
            strong_corr = []
            for i in range(len(correlations.columns)):
                for j in range(i+1, len(correlations.columns)):
                    corr_val = correlations.iloc[i, j]
                    if abs(corr_val) > 0.7:
                        strong_corr.append((
                            correlations.columns[i],
                            correlations.columns[j],
                            corr_val
                        ))
                        
            if strong_corr:
                for var1, var2, corr in strong_corr:
                    report.append(f"  - Strong correlation ({corr:.2f}) between {var1} and {var2}")
            else:
                report.append("  - No strong correlations detected (|r| > 0.7)")
            report.append("\n")
            
        # Anomaly Detection
        if anomalies is not None and not anomalies.empty:
            report.append("ANOMALY DETECTION")
            report.append("-" * 80)
            report.append(f"Total anomalies detected: {len(anomalies)}")
            report.append("\n")
            report.append("Anomaly Details:")
            report.append(self._format_table(anomalies.head(10)))
            if len(anomalies) > 10:
                report.append(f"\n... and {len(anomalies) - 10} more anomalies")
            report.append("\n")
            
        # Vulnerability Testing
        if failures:
            report.append("VULNERABILITY TESTING RESULTS")
            report.append("-" * 80)
            report.append("Failure States Detected:")
            report.append(f"  {failures}")
            report.append("\n")
            report.append("Recommendations:")
            report.append("  1. Implement input validation for velocity commands")
            report.append("  2. Add redundancy checks for critical sensor data")
            report.append("  3. Establish monitoring for abnormal command patterns")
            report.append("  4. Consider implementing rate limiting on command inputs")
            report.append("\n")
            
        # Conclusion
        report.append("CONCLUSION")
        report.append("-" * 80)
        report.append("The analysis identified potential vulnerabilities in the autonomous vehicle")
        report.append("system. Implementing the recommended mitigations will enhance system")
        report.append("robustness against cyber attacks and sensor anomalies.")
        report.append("\n")
        
        report.append("=" * 80)
        report.append("END OF REPORT")
        report.append("=" * 80)
        
        return "\n".join(report)
        
    def _format_table(self, df):
        """
        Convert DataFrame to formatted text table
        
        Args:
            df (pd.DataFrame): DataFrame to format
            
        Returns:
            str: Formatted table string
        """
        if df is None or df.empty:
            return "No data available"
            
        # Convert to string with nice formatting
        return df.to_string()
        
    def generate_vulnerability_summary(self, injection_results):
        """
        Generate summary of vulnerability testing
        
        Args:
            injection_results (list): List of injection test results
            
        Returns:
            str: Summary text
        """
        summary = []
        summary.append("VULNERABILITY TESTING SUMMARY")
        summary.append("-" * 60)
        
        if not injection_results:
            summary.append("No vulnerability tests performed.")
            return "\n".join(summary)
            
        summary.append(f"Total tests performed: {len(injection_results)}")
        
        vulnerabilities_found = sum(1 for r in injection_results if r.get('vulnerable', False))
        summary.append(f"Vulnerabilities identified: {vulnerabilities_found}")
        
        summary.append("\nTest Results:")
        for i, result in enumerate(injection_results, 1):
            summary.append(f"\nTest {i}: {result.get('test_name', 'Unknown')}")
            summary.append(f"  Anomaly Type: {result.get('anomaly_type', 'N/A')}")
            summary.append(f"  Vulnerable: {'Yes' if result.get('vulnerable') else 'No'}")
            if result.get('description'):
                summary.append(f"  Description: {result['description']}")
                
        return "\n".join(summary)
        
    def export_to_csv(self, data, filename):
        """
        Export data to CSV file
        
        Args:
            data (pd.DataFrame): Data to export
            filename (str): Output filename
        """
        if data is not None and not data.empty:
            data.to_csv(filename, index=False)
            return True
        return False
        
    def create_summary_stats(self, df):
        """
        Create summary statistics for DataFrame
        
        Args:
            df (pd.DataFrame): Input data
            
        Returns:
            dict: Summary statistics
        """
        if df is None or df.empty:
            return {}
            
        summary = {
            'row_count': len(df),
            'column_count': len(df.columns),
            'columns': list(df.columns),
            'numeric_summary': {}
        }
        
        # Get numeric column statistics
        numeric_cols = df.select_dtypes(include=['number']).columns
        for col in numeric_cols:
            summary['numeric_summary'][col] = {
                'mean': float(df[col].mean()),
                'std': float(df[col].std()),
                'min': float(df[col].min()),
                'max': float(df[col].max()),
                'median': float(df[col].median())
            }
            
        return summary
