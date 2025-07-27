#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import json
import os
import glob
from datetime import datetime
from scipy import stats
import yaml


class MPCComparativeAnalyzer(Node):
    """
    Comparative analysis tool for MPC controller evaluation results.
    
    This node compares baseline performance with parameter sweep results
    to provide comprehensive insights into controller behavior.
    """
    
    def __init__(self):
        super().__init__('mpc_comparative_analyzer')
        
        # Declare parameters
        self.declare_parameter('baseline_dir', '/tmp/mpc_evaluation/baseline')
        self.declare_parameter('sweep_dir', '/tmp/mpc_evaluation/parameter_sweep')
        self.declare_parameter('output_dir', '/tmp/mpc_evaluation/comparative_analysis')
        self.declare_parameter('statistical_significance', 0.05)
        
        # Get parameters
        self.baseline_dir = self.get_parameter('baseline_dir').value
        self.sweep_dir = self.get_parameter('sweep_dir').value
        self.output_dir = self.get_parameter('output_dir').value
        self.alpha = self.get_parameter('statistical_significance').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load and analyze data
        self.baseline_data = self.load_baseline_data()
        self.sweep_data = self.load_sweep_data()
        
        if self.baseline_data is None or self.sweep_data is None:
            self.get_logger().error("Failed to load required data")
            return
        
        # Perform comparative analysis
        self.run_comparative_analysis()
        
        self.get_logger().info("Comparative analysis completed!")
        
    def load_baseline_data(self):
        """Load baseline experiment data"""
        try:
            # Find most recent baseline summary statistics
            pattern = os.path.join(self.baseline_dir, 'summary_stats_*.json')
            files = glob.glob(pattern)
            
            if not files:
                self.get_logger().error(f"No baseline data found in {self.baseline_dir}")
                return None
            
            latest_file = sorted(files)[-1]
            
            with open(latest_file, 'r') as f:
                baseline_stats = json.load(f)
            
            self.get_logger().info(f"Loaded baseline data from {latest_file}")
            return baseline_stats
            
        except Exception as e:
            self.get_logger().error(f"Error loading baseline data: {e}")
            return None
    
    def load_sweep_data(self):
        """Load parameter sweep data"""
        try:
            # Find sweep results CSV
            pattern = os.path.join(self.sweep_dir, 'sweep_results_*.csv')
            files = glob.glob(pattern)
            
            if not files:
                self.get_logger().error(f"No sweep data found in {self.sweep_dir}")
                return None
            
            latest_file = sorted(files)[-1]
            sweep_df = pd.read_csv(latest_file)
            
            self.get_logger().info(f"Loaded sweep data from {latest_file}")
            return sweep_df
            
        except Exception as e:
            self.get_logger().error(f"Error loading sweep data: {e}")
            return None
    
    def run_comparative_analysis(self):
        """Run comprehensive comparative analysis"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. Statistical comparison with baseline
        statistical_results = self.perform_statistical_analysis()
        
        # 2. Parameter sensitivity analysis
        sensitivity_results = self.analyze_parameter_sensitivity()
        
        # 3. Performance improvement analysis
        improvement_analysis = self.analyze_performance_improvements()
        
        # 4. Robustness analysis
        robustness_analysis = self.analyze_robustness()
        
        # 5. Generate comprehensive plots
        self.generate_comparative_plots(timestamp)
        
        # 6. Generate final report
        self.generate_comparative_report({
            'statistical': statistical_results,
            'sensitivity': sensitivity_results,
            'improvements': improvement_analysis,
            'robustness': robustness_analysis
        }, timestamp)
        
        # 7. Save analysis data
        self.save_analysis_data({
            'statistical': statistical_results,
            'sensitivity': sensitivity_results,
            'improvements': improvement_analysis,
            'robustness': robustness_analysis
        }, timestamp)
    
    def perform_statistical_analysis(self):
        """Perform statistical comparison with baseline"""
        results = {}
        
        # Key metrics to compare
        key_metrics = [
            'position_errors_rms',
            'heading_errors_rms',
            'cross_track_errors_rms',
            'control_effort_mean',
            'control_smoothness_rms'
        ]
        
        for metric in key_metrics:
            if metric in self.baseline_data:
                baseline_value = self.baseline_data[metric]['rms']
                
                if metric in self.sweep_data.columns:
                    sweep_values = self.sweep_data[metric].dropna()
                    
                    # Calculate improvement statistics
                    improvements = (baseline_value - sweep_values) / baseline_value * 100
                    
                    # Statistical tests
                    better_count = (sweep_values < baseline_value).sum()
                    total_count = len(sweep_values)
                    
                    # One-sample t-test against baseline
                    if len(sweep_values) > 1:
                        t_stat, p_value = stats.ttest_1samp(sweep_values, baseline_value)
                    else:
                        t_stat, p_value = 0, 1
                    
                    results[metric] = {
                        'baseline_value': baseline_value,
                        'sweep_mean': float(sweep_values.mean()),
                        'sweep_std': float(sweep_values.std()),
                        'best_improvement': float(improvements.max()),
                        'worst_degradation': float(improvements.min()),
                        'mean_improvement': float(improvements.mean()),
                        'better_than_baseline_count': int(better_count),
                        'better_than_baseline_percent': float(better_count / total_count * 100),
                        't_statistic': float(t_stat),
                        'p_value': float(p_value),
                        'significant_improvement': bool(p_value < self.alpha and sweep_values.mean() < baseline_value)
                    }
        
        return results
    
    def analyze_parameter_sensitivity(self):
        """Analyze parameter sensitivity across all metrics"""
        param_cols = [col for col in self.sweep_data.columns if col.startswith('param_')]
        metric_cols = [col for col in self.sweep_data.columns if '_rms' in col or '_mean' in col]
        
        sensitivity_matrix = {}
        
        for param in param_cols:
            param_name = param.replace('param_', '')
            sensitivity_matrix[param_name] = {}
            
            for metric in metric_cols:
                if param in self.sweep_data.columns and metric in self.sweep_data.columns:
                    # Calculate correlation
                    correlation = self.sweep_data[param].corr(self.sweep_data[metric])
                    
                    # Calculate effect size (normalized)
                    param_range = self.sweep_data[param].max() - self.sweep_data[param].min()
                    metric_range = self.sweep_data[metric].max() - self.sweep_data[metric].min()
                    
                    if param_range > 0 and metric_range > 0:
                        # Linear regression to get slope
                        slope, intercept, r_value, p_value, std_err = stats.linregress(
                            self.sweep_data[param], self.sweep_data[metric])
                        
                        # Normalized effect size
                        effect_size = slope * param_range / metric_range
                    else:
                        effect_size = 0
                        p_value = 1
                    
                    sensitivity_matrix[param_name][metric] = {
                        'correlation': float(correlation) if not np.isnan(correlation) else 0,
                        'effect_size': float(effect_size),
                        'p_value': float(p_value) if not np.isnan(p_value) else 1,
                        'significant': bool(p_value < self.alpha) if not np.isnan(p_value) else False
                    }
        
        return sensitivity_matrix
    
    def analyze_performance_improvements(self):
        """Analyze potential performance improvements"""
        improvements = {}
        
        # Find best performing parameter sets for each metric
        key_metrics = [
            'position_errors_rms',
            'heading_errors_rms', 
            'cross_track_errors_rms',
            'control_effort_mean'
        ]
        
        for metric in key_metrics:
            if metric in self.sweep_data.columns and metric in self.baseline_data:
                baseline_value = self.baseline_data[metric]['rms']
                
                # Find best parameter set for this metric
                best_idx = self.sweep_data[metric].idxmin()
                best_value = self.sweep_data.loc[best_idx, metric]
                
                # Calculate improvement
                improvement = (baseline_value - best_value) / baseline_value * 100
                
                # Get the parameters that achieved this
                best_params = {}
                for col in self.sweep_data.columns:
                    if col.startswith('param_'):
                        best_params[col.replace('param_', '')] = self.sweep_data.loc[best_idx, col]
                
                improvements[metric] = {
                    'baseline_value': baseline_value,
                    'best_value': float(best_value),
                    'improvement_percent': float(improvement),
                    'best_parameters': best_params,
                    'experiment_id': int(self.sweep_data.loc[best_idx, 'experiment_id']) if 'experiment_id' in self.sweep_data.columns else None
                }
        
        # Multi-objective optimization (find balanced improvements)
        if all(metric in self.sweep_data.columns for metric in key_metrics[:3]):
            # Normalize metrics and find balanced solution
            normalized_data = self.sweep_data[key_metrics[:3]].copy()
            for metric in key_metrics[:3]:
                min_val = normalized_data[metric].min()
                max_val = normalized_data[metric].max()
                if max_val > min_val:
                    normalized_data[metric] = (normalized_data[metric] - min_val) / (max_val - min_val)
            
            # Combined score (equal weights)
            combined_score = normalized_data.sum(axis=1)
            balanced_idx = combined_score.idxmin()
            
            balanced_params = {}
            for col in self.sweep_data.columns:
                if col.startswith('param_'):
                    balanced_params[col.replace('param_', '')] = self.sweep_data.loc[balanced_idx, col]
            
            balanced_performance = {}
            for metric in key_metrics[:3]:
                baseline_val = self.baseline_data[metric]['rms']
                best_val = self.sweep_data.loc[balanced_idx, metric]
                balanced_performance[metric] = {
                    'value': float(best_val),
                    'improvement_percent': float((baseline_val - best_val) / baseline_val * 100)
                }
            
            improvements['balanced_solution'] = {
                'parameters': balanced_params,
                'performance': balanced_performance,
                'experiment_id': int(self.sweep_data.loc[balanced_idx, 'experiment_id']) if 'experiment_id' in self.sweep_data.columns else None
            }
        
        return improvements
    
    def analyze_robustness(self):
        """Analyze parameter robustness and consistency"""
        robustness = {}
        
        param_cols = [col for col in self.sweep_data.columns if col.startswith('param_')]
        key_metrics = ['position_errors_rms', 'heading_errors_rms', 'cross_track_errors_rms']
        
        for param in param_cols:
            param_name = param.replace('param_', '')
            
            # Analyze how performance varies with this parameter
            param_values = self.sweep_data[param].unique()
            
            if len(param_values) > 2:  # Need multiple values to analyze robustness
                robustness_data = {}
                
                for metric in key_metrics:
                    if metric in self.sweep_data.columns:
                        # Group by parameter value and calculate statistics
                        grouped = self.sweep_data.groupby(param)[metric]
                        
                        mean_performance = grouped.mean()
                        std_performance = grouped.std()
                        
                        # Calculate robustness metrics
                        performance_range = mean_performance.max() - mean_performance.min()
                        mean_std = std_performance.mean()
                        
                        # Robustness score (lower is more robust)
                        if mean_performance.mean() > 0:
                            robustness_score = (performance_range + mean_std) / mean_performance.mean()
                        else:
                            robustness_score = float('inf')
                        
                        robustness_data[metric] = {
                            'performance_range': float(performance_range),
                            'mean_std': float(mean_std),
                            'robustness_score': float(robustness_score),
                            'most_robust_value': float(param_values[std_performance.idxmin()]) if not std_performance.empty else None
                        }
                
                robustness[param_name] = robustness_data
        
        return robustness
    
    def generate_comparative_plots(self, timestamp):
        """Generate comprehensive comparative plots"""
        # Set up plotting style
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
        # Create main comparison figure
        fig = plt.figure(figsize=(20, 16))
        fig.suptitle('MPC Controller Comparative Analysis', fontsize=16, fontweight='bold')
        
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)
        
        # 1. Performance comparison with baseline
        ax1 = fig.add_subplot(gs[0, :2])
        self.plot_baseline_comparison(ax1)
        
        # 2. Parameter sensitivity heatmap
        ax2 = fig.add_subplot(gs[0, 2:])
        self.plot_sensitivity_heatmap(ax2)
        
        # 3. Improvement distribution
        ax3 = fig.add_subplot(gs[1, :2])
        self.plot_improvement_distribution(ax3)
        
        # 4. Robustness analysis
        ax4 = fig.add_subplot(gs[1, 2:])
        self.plot_robustness_analysis(ax4)
        
        # 5. Parameter correlation matrix
        ax5 = fig.add_subplot(gs[2, :2])
        self.plot_parameter_correlations(ax5)
        
        # 6. Best vs worst comparison
        ax6 = fig.add_subplot(gs[2, 2:])
        self.plot_best_worst_comparison(ax6)
        
        # 7. Performance trade-offs
        ax7 = fig.add_subplot(gs[3, :2])
        self.plot_performance_tradeoffs(ax7)
        
        # 8. Parameter recommendations
        ax8 = fig.add_subplot(gs[3, 2:])
        self.plot_parameter_recommendations(ax8)
        
        plt.savefig(os.path.join(self.output_dir, f'comparative_analysis_{timestamp}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.savefig(os.path.join(self.output_dir, f'comparative_analysis_{timestamp}.pdf'), 
                   bbox_inches='tight')
        plt.close()
    
    def plot_baseline_comparison(self, ax):
        """Plot comparison with baseline performance"""
        metrics = ['position_errors_rms', 'heading_errors_rms', 'cross_track_errors_rms']
        baseline_values = []
        sweep_means = []
        sweep_stds = []
        
        for metric in metrics:
            if metric in self.baseline_data and metric in self.sweep_data.columns:
                baseline_values.append(self.baseline_data[metric]['rms'])
                sweep_means.append(self.sweep_data[metric].mean())
                sweep_stds.append(self.sweep_data[metric].std())
        
        x = np.arange(len(metrics))
        width = 0.35
        
        ax.bar(x - width/2, baseline_values, width, label='Baseline', alpha=0.8)
        ax.bar(x + width/2, sweep_means, width, yerr=sweep_stds, 
               label='Parameter Sweep (Mean ± Std)', alpha=0.8)
        
        ax.set_xlabel('Performance Metrics')
        ax.set_ylabel('Error Values')
        ax.set_title('Performance Comparison: Baseline vs Parameter Sweep')
        ax.set_xticks(x)
        ax.set_xticklabels([m.replace('_', ' ').title() for m in metrics], rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_sensitivity_heatmap(self, ax):
        """Plot parameter sensitivity heatmap"""
        # This is a simplified version - you'd need to process the sensitivity data
        ax.text(0.5, 0.5, 'Parameter Sensitivity\nHeatmap\n(Implementation needed)', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Parameter Sensitivity Analysis')
    
    def plot_improvement_distribution(self, ax):
        """Plot distribution of improvements over baseline"""
        if 'position_errors_rms' in self.sweep_data.columns and 'position_errors_rms' in self.baseline_data:
            baseline_val = self.baseline_data['position_errors_rms']['rms']
            sweep_vals = self.sweep_data['position_errors_rms']
            improvements = (baseline_val - sweep_vals) / baseline_val * 100
            
            ax.hist(improvements, bins=20, alpha=0.7, edgecolor='black')
            ax.axvline(0, color='red', linestyle='--', label='Baseline Performance')
            ax.axvline(improvements.mean(), color='green', linestyle='-', label=f'Mean Improvement: {improvements.mean():.1f}%')
            
            ax.set_xlabel('Improvement over Baseline (%)')
            ax.set_ylabel('Number of Experiments')
            ax.set_title('Distribution of Performance Improvements')
            ax.legend()
            ax.grid(True, alpha=0.3)
    
    def plot_robustness_analysis(self, ax):
        """Plot robustness analysis"""
        ax.text(0.5, 0.5, 'Parameter Robustness\nAnalysis\n(Implementation needed)', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Parameter Robustness Analysis')
    
    def plot_parameter_correlations(self, ax):
        """Plot parameter correlation matrix"""
        param_cols = [col for col in self.sweep_data.columns if col.startswith('param_')]
        if len(param_cols) > 1:
            corr_matrix = self.sweep_data[param_cols].corr()
            sns.heatmap(corr_matrix, annot=True, cmap='RdBu_r', center=0, ax=ax,
                       xticklabels=[col.replace('param_', '') for col in param_cols],
                       yticklabels=[col.replace('param_', '') for col in param_cols])
            ax.set_title('Parameter Correlation Matrix')
    
    def plot_best_worst_comparison(self, ax):
        """Plot best vs worst parameter sets"""
        ax.text(0.5, 0.5, 'Best vs Worst\nParameter Comparison\n(Implementation needed)', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Best vs Worst Parameter Sets')
    
    def plot_performance_tradeoffs(self, ax):
        """Plot performance trade-offs"""
        if ('position_errors_rms' in self.sweep_data.columns and 
            'control_effort_mean' in self.sweep_data.columns):
            
            scatter = ax.scatter(self.sweep_data['position_errors_rms'], 
                               self.sweep_data['control_effort_mean'],
                               alpha=0.6, s=50)
            
            # Mark baseline
            if ('position_errors_rms' in self.baseline_data and 
                'control_effort_mean' in self.baseline_data):
                baseline_pos = self.baseline_data['position_errors_rms']['rms']
                baseline_ctrl = self.baseline_data['control_effort_mean']['rms']
                ax.scatter(baseline_pos, baseline_ctrl, color='red', s=200, 
                          marker='*', label='Baseline')
            
            ax.set_xlabel('Position Error RMS (m)')
            ax.set_ylabel('Control Effort Mean')
            ax.set_title('Performance Trade-offs: Accuracy vs Control Effort')
            ax.legend()
            ax.grid(True, alpha=0.3)
    
    def plot_parameter_recommendations(self, ax):
        """Plot parameter recommendations"""
        ax.text(0.5, 0.5, 'Parameter\nRecommendations\n(Implementation needed)', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Parameter Tuning Recommendations')
    
    def generate_comparative_report(self, analysis_results, timestamp):
        """Generate comprehensive comparative report"""
        report = f"""
# MPC Controller Comparative Analysis Report

**Generated:** {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
**Baseline Directory:** {self.baseline_dir}
**Parameter Sweep Directory:** {self.sweep_dir}

## Executive Summary

This report presents a comprehensive comparative analysis between baseline MPC controller performance and results from systematic parameter variation experiments.

## Statistical Analysis Results

### Performance Comparison with Baseline
"""
        
        if 'statistical' in analysis_results:
            for metric, stats in analysis_results['statistical'].items():
                improvement = stats['mean_improvement']
                significance = "✓" if stats['significant_improvement'] else "✗"
                
                report += f"""
**{metric.replace('_', ' ').title()}:**
- Mean improvement over baseline: {improvement:.2f}%
- Best improvement achieved: {stats['best_improvement']:.2f}%
- Experiments better than baseline: {stats['better_than_baseline_percent']:.1f}%
- Statistical significance: {significance} (p={stats['p_value']:.4f})
"""
        
        # Add more sections for sensitivity, improvements, robustness
        report += """
## Key Findings

1. **Most Significant Improvements:** [To be filled based on analysis]
2. **Most Sensitive Parameters:** [To be filled based on analysis]  
3. **Robustness Insights:** [To be filled based on analysis]
4. **Recommended Parameter Sets:** [To be filled based on analysis]

## Recommendations

### For Production Use
- Use parameter set from experiment [ID] for best overall performance
- Consider parameter set from experiment [ID] for balanced performance
- Monitor [specific parameters] for robustness

### For Further Research  
- Investigate [parameter combinations] showing promise
- Study [robustness issues] identified in analysis
- Consider [additional parameters] not included in current sweep

---
*Report generated by MPC Comparative Analyzer*
"""
        
        # Save the report
        with open(os.path.join(self.output_dir, f'comparative_report_{timestamp}.md'), 'w') as f:
            f.write(report)
    
    def save_analysis_data(self, analysis_results, timestamp):
        """Save analysis data to files"""
        # Save as JSON
        with open(os.path.join(self.output_dir, f'analysis_results_{timestamp}.json'), 'w') as f:
            json.dump(analysis_results, f, indent=2)
        
        # Save statistical results as CSV if available
        if 'statistical' in analysis_results:
            stats_df = pd.DataFrame(analysis_results['statistical']).T
            stats_df.to_csv(os.path.join(self.output_dir, f'statistical_comparison_{timestamp}.csv'))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        analyzer = MPCComparativeAnalyzer()
        # Analysis runs in __init__ and then exits
        
    except KeyboardInterrupt:
        print("Comparative analysis interrupted by user")
    except Exception as e:
        print(f"Error in comparative analysis: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()