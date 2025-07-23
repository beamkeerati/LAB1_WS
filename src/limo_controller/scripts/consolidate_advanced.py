#!/usr/bin/python3
"""
Tracking Data Consolidator

This script combines multiple tracking data YAML files into a single comprehensive report.
It merges all tracking data points, recalculates statistics, and creates a unified analysis.
"""

import os
import yaml
import numpy as np
import math
import glob
from datetime import datetime
from collections import defaultdict
import argparse


class TrackingDataConsolidator:
    def __init__(self, input_directory, output_file=None):
        self.input_directory = input_directory
        self.output_file = output_file or "consolidated_tracking_results.yaml"
        self.all_tracking_data = []
        self.all_statistics = defaultdict(list)
        self.metadata = {
            'files_processed': 0,
            'total_data_points': 0,
            'time_range': {'start': None, 'end': None},
            'experiment_duration': 0,
            'processing_timestamp': datetime.now().strftime("%Y%m%d_%H%M%S")
        }
        
    def find_yaml_files(self):
        """Find all tracking data YAML files in the directory"""
        pattern = os.path.join(self.input_directory, "tracking_data_*.yaml")
        files = glob.glob(pattern)
        
        # Also include final results files
        final_pattern = os.path.join(self.input_directory, "final_tracking_results_*.yaml")
        files.extend(glob.glob(final_pattern))
        
        # Sort files by timestamp for chronological processing
        files.sort()
        return files
    
    def load_yaml_file(self, filepath):
        """Load and parse a YAML file"""
        try:
            with open(filepath, 'r') as file:
                data = yaml.safe_load(file)
            return data
        except Exception as e:
            print(f"Error loading {filepath}: {e}")
            return None
    
    def extract_tracking_data(self, data):
        """Extract tracking data points from a loaded YAML structure"""
        tracking_points = []
        
        # Handle different file structures
        if 'tracking_data' in data:
            tracking_points.extend(data['tracking_data'])
        elif 'full_tracking_data' in data:
            tracking_points.extend(data['full_tracking_data'])
        elif isinstance(data, list):
            # Sometimes the root is directly a list of tracking points
            tracking_points.extend(data)
        
        return tracking_points
    
    def extract_statistics(self, data):
        """Extract statistics from a loaded YAML structure"""
        stats = {}
        if 'statistics' in data:
            stats = data['statistics']
        elif 'performance_statistics' in data:
            stats = data['performance_statistics']
        return stats
    
    def merge_tracking_data(self, tracking_points):
        """Add tracking points to the consolidated dataset"""
        for point in tracking_points:
            # Ensure consistent timestamp format
            if 'timestamp' in point:
                self.all_tracking_data.append(point)
    
    def update_time_range(self, tracking_points):
        """Update the time range based on tracking data timestamps"""
        for point in tracking_points:
            if 'timestamp' in point:
                timestamp = point['timestamp']
                if isinstance(timestamp, dict) and 'sec' in timestamp:
                    # Convert ROS timestamp to datetime
                    time_val = timestamp['sec'] + timestamp.get('nanosec', 0) / 1e9
                    if self.metadata['time_range']['start'] is None or time_val < self.metadata['time_range']['start']:
                        self.metadata['time_range']['start'] = time_val
                    if self.metadata['time_range']['end'] is None or time_val > self.metadata['time_range']['end']:
                        self.metadata['time_range']['end'] = time_val
    
    def calculate_comprehensive_statistics(self):
        """Calculate comprehensive statistics from all tracking data"""
        if not self.all_tracking_data:
            return {}
        
        # Collect all error values
        errors = {
            'lateral_error': [],
            'heading_error': [],
            'speed_error': [],
            'position_error': [],
            'along_track_error': []
        }
        
        for point in self.all_tracking_data:
            if 'tracking_errors' in point:
                for error_type in errors.keys():
                    if error_type in point['tracking_errors']:
                        value = point['tracking_errors'][error_type]
                        if value is not None and not math.isnan(value):
                            errors[error_type].append(value)
        
        # Calculate comprehensive statistics
        comprehensive_stats = {}
        for error_type, values in errors.items():
            if values:
                values_array = np.array(values)
                comprehensive_stats[error_type] = {
                    'mean': float(np.mean(values_array)),
                    'std': float(np.std(values_array)),
                    'max': float(np.max(values_array)),
                    'min': float(np.min(values_array)),
                    'rms': float(np.sqrt(np.mean(np.square(values_array)))),
                    'median': float(np.median(values_array)),
                    'percentile_95': float(np.percentile(values_array, 95)),
                    'percentile_99': float(np.percentile(values_array, 99)),
                    'count': len(values)
                }
        
        return comprehensive_stats
    
    def process_all_files(self):
        """Process all YAML files and consolidate the data"""
        yaml_files = self.find_yaml_files()
        print(f"Found {len(yaml_files)} YAML files to process")
        
        for filepath in yaml_files:
            print(f"Processing: {os.path.basename(filepath)}")
            data = self.load_yaml_file(filepath)
            
            if data is None:
                continue
            
            # Extract and merge tracking data
            tracking_points = self.extract_tracking_data(data)
            if tracking_points:
                self.merge_tracking_data(tracking_points)
                self.update_time_range(tracking_points)
                self.metadata['total_data_points'] += len(tracking_points)
            
            # Extract statistics for comparison
            stats = self.extract_statistics(data)
            if stats:
                for stat_type, stat_values in stats.items():
                    if isinstance(stat_values, dict):
                        self.all_statistics[stat_type].append(stat_values)
            
            self.metadata['files_processed'] += 1
        
        # Calculate experiment duration
        if self.metadata['time_range']['start'] and self.metadata['time_range']['end']:
            self.metadata['experiment_duration'] = (
                self.metadata['time_range']['end'] - self.metadata['time_range']['start']
            )
        
        print(f"Consolidated {self.metadata['total_data_points']} tracking data points from {self.metadata['files_processed']} files")
    
    def create_summary_analysis(self):
        """Create a high-level summary analysis"""
        stats = self.calculate_comprehensive_statistics()
        
        summary = {
            'experiment_overview': {
                'total_files_processed': self.metadata['files_processed'],
                'total_data_points': self.metadata['total_data_points'],
                'experiment_duration_seconds': self.metadata['experiment_duration'],
                'experiment_duration_minutes': self.metadata['experiment_duration'] / 60.0,
                'average_sampling_rate_hz': self.metadata['total_data_points'] / self.metadata['experiment_duration'] if self.metadata['experiment_duration'] > 0 else 0,
                'processing_date': self.metadata['processing_timestamp']
            },
            'performance_summary': {},
            'tracking_quality_assessment': {}
        }
        
        if 'lateral_error' in stats:
            summary['performance_summary'] = {
                'lateral_tracking': {
                    'rms_error_m': stats['lateral_error']['rms'],
                    'max_error_m': stats['lateral_error']['max'],
                    'mean_error_m': stats['lateral_error']['mean'],
                    'std_error_m': stats['lateral_error']['std']
                },
                'heading_tracking': {
                    'rms_error_deg': math.degrees(stats['heading_error']['rms']),
                    'max_error_deg': math.degrees(stats['heading_error']['max']),
                    'mean_error_deg': math.degrees(stats['heading_error']['mean']),
                    'std_error_deg': math.degrees(stats['heading_error']['std'])
                },
                'position_tracking': {
                    'rms_error_m': stats['position_error']['rms'],
                    'max_error_m': stats['position_error']['max'],
                    'mean_error_m': stats['position_error']['mean'],
                    'std_error_m': stats['position_error']['std']
                }
            }
            
            # Quality assessment
            lateral_rms = stats['lateral_error']['rms']
            heading_rms_deg = math.degrees(stats['heading_error']['rms'])
            
            if lateral_rms < 0.1:
                lateral_quality = "Excellent"
            elif lateral_rms < 0.2:
                lateral_quality = "Good"
            elif lateral_rms < 0.5:
                lateral_quality = "Fair"
            else:
                lateral_quality = "Poor"
            
            if heading_rms_deg < 5:
                heading_quality = "Excellent"
            elif heading_rms_deg < 10:
                heading_quality = "Good"
            elif heading_rms_deg < 20:
                heading_quality = "Fair"
            else:
                heading_quality = "Poor"
            
            summary['tracking_quality_assessment'] = {
                'lateral_tracking_quality': lateral_quality,
                'heading_tracking_quality': heading_quality,
                'overall_assessment': "Good" if lateral_quality in ["Excellent", "Good"] and heading_quality in ["Excellent", "Good"] else "Needs Improvement"
            }
        
        return summary
    
    def save_consolidated_report(self):
        """Save the consolidated report to a YAML file"""
        # Calculate final statistics
        comprehensive_stats = self.calculate_comprehensive_statistics()
        summary = self.create_summary_analysis()
        
        # Create the consolidated report structure
        consolidated_report = {
            'consolidation_metadata': {
                'generated_at': self.metadata['processing_timestamp'],
                'source_directory': self.input_directory,
                'files_processed': self.metadata['files_processed'],
                'consolidation_method': 'complete_merge',
                'script_version': '1.0'
            },
            'experiment_summary': summary,
            'detailed_statistics': comprehensive_stats,
            'raw_tracking_data': self.all_tracking_data[-1000:] if len(self.all_tracking_data) > 1000 else self.all_tracking_data,  # Keep last 1000 points to avoid huge files
            'data_overview': {
                'total_tracking_points': len(self.all_tracking_data),
                'time_range': self.metadata['time_range'],
                'experiment_duration_seconds': self.metadata['experiment_duration']
            }
        }
        
        # Save to file
        output_path = os.path.join(self.input_directory, self.output_file)
        try:
            with open(output_path, 'w') as file:
                yaml.dump(consolidated_report, file, default_flow_style=False, indent=2)
            print(f"Consolidated report saved to: {output_path}")
            return output_path
        except Exception as e:
            print(f"Error saving consolidated report: {e}")
            return None
    
    def print_summary(self):
        """Print a summary of the consolidation process"""
        stats = self.calculate_comprehensive_statistics()
        
        print("\n" + "="*60)
        print("TRACKING DATA CONSOLIDATION SUMMARY")
        print("="*60)
        print(f"Files Processed: {self.metadata['files_processed']}")
        print(f"Total Data Points: {self.metadata['total_data_points']}")
        print(f"Experiment Duration: {self.metadata['experiment_duration']:.1f} seconds ({self.metadata['experiment_duration']/60:.1f} minutes)")
        
        if 'lateral_error' in stats:
            print(f"\nTRACKING PERFORMANCE:")
            print(f"  Lateral Error (RMS): {stats['lateral_error']['rms']:.3f} m")
            print(f"  Lateral Error (Max): {stats['lateral_error']['max']:.3f} m")
            print(f"  Heading Error (RMS): {math.degrees(stats['heading_error']['rms']):.1f} degrees")
            print(f"  Heading Error (Max): {math.degrees(stats['heading_error']['max']):.1f} degrees")
            print(f"  Position Error (RMS): {stats['position_error']['rms']:.3f} m")
            print(f"  Position Error (Max): {stats['position_error']['max']:.3f} m")
        
        print("="*60)


def main():
    parser = argparse.ArgumentParser(description='Consolidate tracking data YAML files')
    parser.add_argument('input_directory', help='Directory containing tracking data YAML files')
    parser.add_argument('-o', '--output', default='consolidated_tracking_results.yaml', 
                       help='Output filename (default: consolidated_tracking_results.yaml)')
    parser.add_argument('--summary-only', action='store_true', 
                       help='Only print summary without saving file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input_directory):
        print(f"Error: Directory {args.input_directory} does not exist")
        return 1
    
    # Create consolidator and process files
    consolidator = TrackingDataConsolidator(args.input_directory, args.output)
    consolidator.process_all_files()
    
    # Print summary
    consolidator.print_summary()
    
    # Save consolidated report unless summary-only
    if not args.summary_only:
        output_path = consolidator.save_consolidated_report()
        if output_path:
            print(f"\nConsolidated report saved to: {output_path}")
        else:
            print("\nFailed to save consolidated report")
            return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
